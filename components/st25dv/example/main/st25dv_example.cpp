#include <array>
#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include <esp_err.h>
#include <esp_log.h>

#include <esp_bt.h>
#include <esp_bt_defs.h>
#include <esp_bt_device.h>
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#if CONFIG_BT_BLE_ENABLED
#include <esp_gap_ble_api.h>
#include <esp_gatt_defs.h>
#include <esp_gattc_api.h>
#endif
#include <nvs_flash.h>

#include "i2c.hpp"
#include "st25dv.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

static constexpr auto HIDD_IDLE_MODE = 0x00;
static constexpr auto HIDD_BLE_MODE = 0x01;
static constexpr auto HIDD_BT_MODE = 0x02;
static constexpr auto HIDD_BTDM_MODE = 0x03;

#if CONFIG_BT_HID_DEVICE_ENABLED
#if CONFIG_BT_BLE_ENABLED
static constexpr auto HID_DEV_MODE = HIDD_BTDM_MODE;
#else
static constexpr auto HID_DEV_MODE = HIDD_BT_MODE;
#endif
#elif CONFIG_BT_BLE_ENABLED
static constexpr auto HID_DEV_MODE = HIDD_BLE_MODE;
#else
static constexpr auto HID_DEV_MODE = HIDD_IDLE_MODE;
#endif

static esp_err_t init_low_level(uint8_t mode);

extern "C" void app_main(void) {
  static auto start = std::chrono::high_resolution_clock::now();
  static auto elapsed = [&]() {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<float>(now - start).count();
  };

  // Initialize NVS - needed for bluetooth
  auto ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  {
    std::atomic<bool> quit_test = false;
    fmt::print("Starting st25dv example, place your phone near it (while running NFC Tools app) to "
               "quit!\n");
    //! [st25dv example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
        .clk_speed = 1000 * 1000,
    });

    // now make the st25dv which decodes the data
    espp::St25dv st25dv({.write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1,
                                            std::placeholders::_2, std::placeholders::_3),
                         .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1,
                                           std::placeholders::_2, std::placeholders::_3),
                         .log_level = espp::Logger::Verbosity::INFO});

    std::array<uint8_t, 50> programmed_data;
    std::error_code ec;
    st25dv.read(programmed_data.data(), programmed_data.size(), ec);
    if (ec) {
      fmt::print("Failed to read st25dv: {}\n", ec.message());
      return;
    }
    fmt::print("Read: {}\n", programmed_data);

    std::vector<espp::Ndef> records;

    // create some sample records
    int payload_id = '0';

    records.emplace_back(espp::Ndef::make_handover_select(payload_id));
    records.emplace_back(espp::Ndef::make_text("hello!"));
    records.emplace_back(espp::Ndef::make_uri("github.com/esp-cpp/espp", espp::Ndef::Uic::HTTPS));
    records.emplace_back(espp::Ndef::make_android_launcher("com.google.android.apps.photos"));
    records.emplace_back(espp::Ndef::make_wifi_config({
        .ssid = CONFIG_ESP_WIFI_SSID,
        .key = CONFIG_ESP_WIFI_PASSWORD,
    }));

    // create BLE OOB pairing record
    uint64_t radio_mac_addr = 0x060504030201; // 48b, example address 06:05:04:03:02:01
#if CONFIG_BT_BLE_ENABLED
    // get the mac address of the radio
    init_low_level(HID_DEV_MODE);
    const uint8_t *point = esp_bt_dev_get_address();
    if (point == nullptr) {
      fmt::print("Failed to get radio mac address!\n");
      return;
    } else {
      // convert the 6 byte mac address to a 48 bit integer
      for (int i = 0; i < 6; i++) {
        radio_mac_addr |= (uint64_t)point[5 - i] << (i * 8);
      }
    }
    fmt::print("radio mac addr: {:#x}\n", radio_mac_addr);
#endif
    auto ble_role = espp::Ndef::BleRole::PERIPHERAL_ONLY;
    auto ble_appearance = espp::Ndef::BtAppearance::GAMEPAD;
    std::string_view ble_radio_name = "BLE Radio";
    records.emplace_back(
        espp::Ndef::make_le_oob_pairing(radio_mac_addr, ble_role, ble_radio_name, ble_appearance));
    records.back().set_id(payload_id);

    // set one of the records we made to be the active tag
    st25dv.set_records(records, ec);

    // and finally, make the task to periodically poll the st25dv and print the
    // state. The task will trigger sample quit when the phone reads the tag.
    auto task_fn = [&quit_test, &st25dv](std::mutex &m, std::condition_variable &cv) {
      {
        std::unique_lock<std::mutex> lock(m);
        cv.wait_for(lock, 30ms);
      }
      std::error_code ec;
      auto it_sts = st25dv.get_interrupt_status(ec);
      if (ec) {
        fmt::print("Failed to get interrupt status: {}\n", ec.message());
        // wait a bit before trying again
        std::unique_lock<std::mutex> lock(m);
        cv.wait_for(lock, 300ms);
        return false;
      }
      static auto last_it_sts = it_sts;
      if (it_sts != last_it_sts) {
        fmt::print("[{:.3f}] IT STS: {}\n", elapsed(), it_sts);
      }
      last_it_sts = it_sts;
      // we don't want to stop the task, so return false
      return false;
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config =
                                {
                                    .name = "St25dv Task",
                                    .stack_size_bytes = 5 * 1024,
                                },
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [st25dv example]
    while (!quit_test) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("St25dv example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

static esp_err_t init_low_level(uint8_t mode) {
  esp_err_t ret;
#if CONFIG_BT_BLE_ENABLED
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
#if CONFIG_IDF_TARGET_ESP32
  bt_cfg.mode = mode;
#endif
  {
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
      fmt::print("esp_bt_controller_mem_release failed: {}\n", ret);
      return ret;
    }
  }
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    fmt::print("esp_bt_controller_init failed: {}\n", ret);
    return ret;
  }

  ret = esp_bt_controller_enable((esp_bt_mode_t)mode);
  if (ret) {
    fmt::print("esp_bt_controller_enable failed: {}\n", ret);
    return ret;
  }

  ret = esp_bluedroid_init();
  if (ret) {
    fmt::print("esp_bluedroid_init failed: {}\n", ret);
    return ret;
  }

  ret = esp_bluedroid_enable();
  if (ret) {
    fmt::print("esp_bluedroid_enable failed: {}\n", ret);
    return ret;
  }
#endif
  return ret;
}
