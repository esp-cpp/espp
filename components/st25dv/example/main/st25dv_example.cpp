#include <array>
#include <chrono>
#include <vector>

#include "driver/i2c.h"

#include "task.hpp"
#include "st25dv.hpp"

using namespace std::chrono_literals;

#define I2C_NUM         (I2C_NUM_1)
#define I2C_SCL_IO      (GPIO_NUM_40)
#define I2C_SDA_IO      (GPIO_NUM_41)
#define I2C_FREQ_HZ     (400 * 1000)
#define I2C_TIMEOUT_MS  (10)

extern "C" void app_main(void) {
  {
    std::atomic<bool> quit_test = false;
    fmt::print("Starting st25dv example, place your phone near it (while running NFC Tools app) to quit!\n");
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
    if (err != ESP_OK) printf("config i2c failed\n");
    err = i2c_driver_install(I2C_NUM, I2C_MODE_MASTER,  0, 0, 0);
    if (err != ESP_OK) printf("install i2c driver failed\n");
    // make some lambda functions we'll use to read/write to the st25dv
    auto st25dv_write = [](uint8_t addr, uint8_t* data, uint8_t length) {
      i2c_master_write_to_device(I2C_NUM,
                                 addr,
                                 data,
                                 length,
                                 I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    };

    auto st25dv_read = [](uint8_t addr, uint16_t reg_addr, uint8_t *data, uint8_t length) {
      uint8_t reg[2] = {
        (uint8_t)(reg_addr >> 8),
        (uint8_t)(reg_addr & 0xFF)
      };
      i2c_master_write_read_device(I2C_NUM,
                                   addr,
                                   reg,
                                   2,
                                   data,
                                   length,
                                   I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    };
    // now make the st25dv which decodes the data
    espp::St25dv st25dv({
        .write = st25dv_write,
        .read = st25dv_read,
        .log_level = espp::Logger::Verbosity::DEBUG
      });

    std::array<uint8_t, 50> programmed_data;
    st25dv.read(programmed_data.data(), programmed_data.size());
    fmt::print("Read: {}\n", programmed_data);

    auto text_record = espp::Ndef::make_text("hello!");
    auto uri_record = espp::Ndef::make_uri("github.com/esp-cpp/espp", espp::Ndef::Uic::HTTPS);
    auto launcher_record = espp::Ndef::make_android_launcher("com.google.android.apps.photos");
    uint64_t radio_mac_addr = 0xFEDCBA100908; // 48b
    uint32_t bt_device_class = 0x000000; // 24b
    std::string_view bt_radio_name = "BT Radio";
    auto bt_oob_record = espp::Ndef::make_oob_pairing(radio_mac_addr, bt_device_class, bt_radio_name);
    auto ble_role = espp::Ndef::BleRole::LE_ROLE_PERIPHERAL_ONLY;
    std::string_view ble_radio_name = "BLE Radio";
    auto ble_oob_record = espp::Ndef::make_le_oob_pairing(radio_mac_addr, ble_role, ble_radio_name);
    st25dv.set_record(bt_oob_record);
    fmt::print("text: {::#x}\n", text_record.serialize());
    fmt::print("uri:  {::#x}\n", uri_record.serialize());
    fmt::print("launcher:  {::#x}\n", launcher_record.serialize());
    fmt::print("bt oob:   {::#x}\n", bt_oob_record.serialize());
    fmt::print("ble oob:  {::#x}\n", ble_oob_record.serialize());
    // and finally, make the task to periodically poll the st25dv and print the
    // state.
    auto task_fn = [&quit_test, &st25dv](std::mutex& m, std::condition_variable& cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds = std::chrono::duration<float>(now-start).count();
      auto it_sts = st25dv.get_interrupt_status();
      fmt::print("IT STS: {:02x}\n", it_sts);
      quit_test = it_sts && espp::St25dv::IT_STS::FIELD_RISING;
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 200ms);
      }
    };
    auto task = espp::Task({
        .name = "St25dv Task",
        .callback = task_fn,
        .stack_size_bytes = 5*1024,
        .log_level = espp::Logger::Verbosity::WARN
      });
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
