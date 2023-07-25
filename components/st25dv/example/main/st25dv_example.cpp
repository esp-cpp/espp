#include <array>
#include <chrono>
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

#include <driver/i2c.h>

#include "st25dv.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

static constexpr auto I2C_NUM = I2C_NUM_1;
static constexpr auto I2C_SCL_IO = GPIO_NUM_40;
static constexpr auto I2C_SDA_IO = GPIO_NUM_41;
static constexpr auto I2C_FREQ_HZ = (400 * 1000);
static constexpr auto I2C_TIMEOUT_MS = 10;

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
    i2c_config_t i2c_cfg;
    fmt::print("initializing i2c driver...\n");
    memset(&i2c_cfg, 0, sizeof(i2c_cfg));
    i2c_cfg.sda_io_num = I2C_SDA_IO;
    i2c_cfg.scl_io_num = I2C_SCL_IO;
    i2c_cfg.mode = I2C_MODE_MASTER;
    i2c_cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_cfg.master.clk_speed = I2C_FREQ_HZ;
    auto err = i2c_param_config(I2C_NUM, &i2c_cfg);
    if (err != ESP_OK)
      printf("config i2c failed\n");
    err = i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK)
      printf("install i2c driver failed\n");
    // make some lambda functions we'll use to read/write to the st25dv
    auto st25dv_write = [](uint8_t addr, uint8_t *data, uint8_t length) {
      i2c_master_write_to_device(I2C_NUM, addr, data, length, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    };

    auto st25dv_read = [](uint8_t addr, uint16_t reg_addr, uint8_t *data, uint8_t length) {
      uint8_t reg[2] = {(uint8_t)(reg_addr >> 8), (uint8_t)(reg_addr & 0xFF)};
      i2c_master_write_read_device(I2C_NUM, addr, reg, 2, data, length,
                                   I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    };
    // now make the st25dv which decodes the data
    espp::St25dv st25dv(
        {.write = st25dv_write, .read = st25dv_read, .log_level = espp::Logger::Verbosity::DEBUG});

    std::array<uint8_t, 50> programmed_data;
    st25dv.read(programmed_data.data(), programmed_data.size());
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
    st25dv.set_records(records);

    // and finally, make the task to periodically poll the st25dv and print the
    // state. The task will trigger sample quit when the phone reads the tag.
    auto task_fn = [&quit_test, &st25dv](std::mutex &m, std::condition_variable &cv) {
      auto it_sts = st25dv.get_interrupt_status();
      static auto last_it_sts = it_sts;
      if (it_sts != last_it_sts) {
        fmt::print("[{:.3f}] IT STS: {:02x}\n", elapsed(), it_sts);
      }
      last_it_sts = it_sts;
      std::unique_lock<std::mutex> lock(m);
      cv.wait_for(lock, 10ms);
      // we don't want to stop the task, so return false
      return false;
    };
    auto task = espp::Task({.name = "St25dv Task",
                            .callback = task_fn,
                            .stack_size_bytes = 5 * 1024,
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [st25dv example]
    while (!quit_test) {
      std::this_thread::sleep_for(100ms);
    }
    // now clean up the i2c driver
    i2c_driver_delete(I2C_NUM);
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
