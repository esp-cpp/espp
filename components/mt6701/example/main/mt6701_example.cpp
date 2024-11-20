#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include <driver/spi_master.h>

#include "butterworth_filter.hpp"
#include "i2c.hpp"
#include "mt6701.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  fmt::print("Starting mt6701 example, rotate to -720 degrees to quit!\n");
#if CONFIG_EXAMPLE_RUN_I2C
  {
    fmt::print("Running I2C example\n");
    //! [mt6701 i2c example]
    std::atomic<bool> quit_test = false;

    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
        .clk_speed = 1 * 1000 * 1000, // MT6701 supports 1 MHz I2C
    });

    // make the velocity filter
    static constexpr float filter_cutoff_hz = 10.0f;
    static constexpr float encoder_update_period = 0.001f; // seconds
    espp::ButterworthFilter<2, espp::BiquadFilterDf2> filter(
        {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * encoder_update_period});
    auto filter_fn = [&filter](float raw) -> float { return filter.update(raw); };

    // now make the mt6701 which decodes the data
    using Mt6701 = espp::Mt6701<espp::Mt6701Interface::I2C>;
    Mt6701 mt6701(
        Mt6701::Config{.write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1,
                                          std::placeholders::_2, std::placeholders::_3),
                       .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1,
                                         std::placeholders::_2, std::placeholders::_3),
                       .velocity_filter = filter_fn,
                       .update_period = std::chrono::duration<float>(encoder_update_period),
                       .run_task = true, // run a task which calls the update function at the update
                                         // period
                       .log_level = espp::Logger::Verbosity::WARN});

    // NOTE: since this is I2C, we cannot get the magnetic field strength,
    // tracking status, or push button state

    // and finally, make the task to periodically poll the mt6701 and print the
    // state. NOTE: the Mt6701 runs its own task to maintain state, so we're
    // just polling the current state.
    auto task_fn = [&quit_test, &mt6701](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds = std::chrono::duration<float>(now - start).count();
      auto count = mt6701.get_count();
      auto radians = mt6701.get_radians();
      auto degrees = mt6701.get_degrees();
      auto rpm = mt6701.get_rpm();
      fmt::print("{:.3f}, {}, {:.3f}, {:.3f}, {:.3f}\n", seconds, count, radians, degrees, rpm);
      quit_test = degrees <= -720.0f;
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 10ms);
      }
      // don't want to stop the task
      return false;
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config =
                                {
                                    .name = "Mt6701 Task",
                                    .stack_size_bytes = 5 * 1024,
                                },
                            .log_level = espp::Logger::Verbosity::WARN});
    fmt::print("% time(s), count, radians, degrees, rpm\n");
    task.start();
    //! [mt6701 i2c example]

    while (!quit_test) {
      std::this_thread::sleep_for(100ms);
    }
  }
#endif // CONFIG_EXAMPLE_RUN_I2C

#if CONFIG_EXAMPLE_RUN_SPI
  {
    fmt::print("Running SPI example\n");
    //! [mt6701 ssi example]
    std::atomic<bool> quit_test = false;

    // make the SSI (SPI) that we'll use to communicate

    // create the spi host
    spi_device_handle_t encoder_spi_handle;
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.mosi_io_num = -1;
    buscfg.miso_io_num = CONFIG_EXAMPLE_SPI_MISO_GPIO;
    buscfg.sclk_io_num = CONFIG_EXAMPLE_SPI_SCLK_GPIO;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 32;

    // create the spi device
    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.mode = 0;
    devcfg.clock_speed_hz = CONFIG_EXAMPLE_SPI_CLOCK_SPEED; // Supports 64ns clock period, 15.625MHz
    devcfg.input_delay_ns = 0;
    devcfg.spics_io_num = CONFIG_EXAMPLE_SPI_CS_GPIO;
    devcfg.queue_size = 1;

    esp_err_t ret;
    // Initialize the SPI bus
    auto spi_num = SPI2_HOST;
    ret = spi_bus_initialize(spi_num, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(spi_num, &devcfg, &encoder_spi_handle);
    ESP_ERROR_CHECK(ret);

    // make the velocity filter
    static constexpr float filter_cutoff_hz = 10.0f;
    static constexpr float encoder_update_period = 0.001f; // seconds
    espp::ButterworthFilter<2, espp::BiquadFilterDf2> filter(
        {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * encoder_update_period});
    auto filter_fn = [&filter](float raw) -> float { return filter.update(raw); };

    // now make the mt6701 which decodes the data
    using Mt6701 = espp::Mt6701<espp::Mt6701Interface::SSI>;
    Mt6701 mt6701({.read = [&](uint8_t *data, size_t len) -> bool {
                     // we can use the SPI_TRANS_USE_RXDATA since our length is <= 4 bytes (32
                     // bits), this means we can directly use the tarnsaction's rx_data field
                     static constexpr uint8_t SPIBUS_READ = 0x80;
                     spi_transaction_t t = {
                         .flags = 0,
                         .cmd = 0,
                         .addr = SPIBUS_READ,
                         .length = len * 8,
                         .rxlength = len * 8,
                         .user = nullptr,
                         .tx_buffer = nullptr,
                         .rx_buffer = data,
                     };
                     if (len <= 4) {
                       t.flags = SPI_TRANS_USE_RXDATA;
                       t.rx_buffer = nullptr;
                     }
                     esp_err_t err = spi_device_transmit(encoder_spi_handle, &t);
                     if (err != ESP_OK) {
                       return false;
                     }
                     if (len <= 4) {
                       // copy the data from the rx_data field
                       for (size_t i = 0; i < len; i++) {
                         data[i] = t.rx_data[i];
                       }
                     }
                     return true;
                   },
                   .velocity_filter = filter_fn,
                   .update_period = std::chrono::duration<float>(encoder_update_period),
                   .run_task = true, // run a task which calls the update function at the update
                                     // period
                   .log_level = espp::Logger::Verbosity::WARN});

    // get the initial state
    auto field_strength = mt6701.get_magnetic_field_strength();
    auto tracking_status = mt6701.get_tracking_status();
    bool push_button = mt6701.get_push_button();
    fmt::print("Initial state: field_strength: {}, tracking_status: {}, push_button: {}\n",
               field_strength, tracking_status, push_button);

    // and finally, make the task to periodically poll the mt6701 and print the
    // state. NOTE: the Mt6701 runs its own task to maintain state, so we're
    // just polling the current state.
    auto task_fn = [&quit_test, &mt6701](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds = std::chrono::duration<float>(now - start).count();
      auto count = mt6701.get_count();
      auto radians = mt6701.get_radians();
      auto degrees = mt6701.get_degrees();
      auto rpm = mt6701.get_rpm();
      fmt::print("{:.3f}, {}, {:.3f}, {:.3f}, {:.3f}\n", seconds, count, radians, degrees, rpm);
      quit_test = degrees <= -720.0f;
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 10ms);
      }
      // don't want to stop the task
      return false;
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config =
                                {
                                    .name = "Mt6701 Task",
                                    .stack_size_bytes = 5 * 1024,
                                },
                            .log_level = espp::Logger::Verbosity::WARN});
    fmt::print("% time(s), count, radians, degrees, rpm\n");
    task.start();
    //! [mt6701 ssi example]

    while (!quit_test) {
      std::this_thread::sleep_for(100ms);
    }
  }
#endif // CONFIG_EXAMPLE_RUN_SPI

  fmt::print("Mt6701 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
