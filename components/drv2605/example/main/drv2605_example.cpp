#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "drv2605.hpp"
#include "i2c.hpp"
#include "logger.hpp"
#include "task.hpp"

#if CONFIG_COMPILER_CXX_EXCEPTIONS
#include "cli.hpp"
#include "drv2605_menu.hpp"
#endif

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "drv2605 example", .level = espp::Logger::Verbosity::INFO});
  // This example shows using the i2c haptic motor driver (drv2605)
  {
    logger.info("Starting!");
    //! [drv2605 example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_1,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
    });

    using Driver = espp::Drv2605;

    // make the actual drv2605 class
    Driver drv2605(Driver::Config{
        .device_address = Driver::DEFAULT_ADDRESS,
        .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                           std::placeholders::_3),
        .read_register =
            std::bind(&espp::I2c::read_at_register, &i2c, std::placeholders::_1,
                      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
        .motor_type = Driver::MotorType::LRA,
        .log_level = espp::Logger::Verbosity::INFO,
    });
    std::error_code ec;
    // we're using an ERM motor, so select an ERM library (1-5).
    // drv2605.select_library(1, ec);
    // we're using an LRA motor, so select an LRA library (6).
    drv2605.select_library(Driver::Library::LRA, ec);
    if (ec) {
      logger.error("select library failed: {}", ec.message());
    }

#if CONFIG_COMPILER_CXX_EXCEPTIONS
    //! [drv2605 menu example]
    using DriverPtr = std::shared_ptr<Driver>;
    // since we're wrapping a stack-allocated object, we need to ensure that the
    // shared pointer does not take ownership of the object and won't delete it
    // when it goes out of scope. There are a couple of ways of doing this, but
    // this method doesn't allocate a control block and is noexcept.
    auto drv2605_ptr = DriverPtr(DriverPtr{}, &drv2605); // no ownership, won't delete
    espp::Drv2605Menu drv2605_menu({drv2605_ptr});
    cli::Cli cli(drv2605_menu.get());
    cli::SetColor();
    cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });
    espp::Cli input(cli);
    input.SetInputHistorySize(10);
    // As this is in the primary thread, we hold here until cli
    // is complete. This is a blocking call and will not return until
    // the user enters the `exit` command.
    input.Start();
    //! [drv2605 menu example]
#endif

    // do the calibration for the LRA motor
    Driver::LraCalibrationSettings lra_calibration_settings{};
    lra_calibration_settings.rated_voltage = 255;
    lra_calibration_settings.overdrive_clamp = 255;
    lra_calibration_settings.drive_time = Driver::lra_freq_to_drive_time(200.0f);
    Driver::CalibratedData calibrated_data;
    if (!drv2605.calibrate(lra_calibration_settings, calibrated_data, ec)) {
      logger.error("calibration failed: {}", ec.message());
      return;
    }
    logger.info("calibration complete: {}", calibrated_data);

    // make the task which will cycle through all the waveforms
    auto task_fn = [&drv2605](std::mutex &m, std::condition_variable &cv) {
      static uint8_t waveform = 0;
      std::error_code ec;
      drv2605.stop(ec);
      if (ec) {
        logger.error("stop failed: {}", ec.message());
        return false;
      }
      drv2605.set_waveform(0, (Driver::Waveform)waveform, ec);
      if (ec) {
        logger.error("set waveform failed: {}", ec.message());
        return false;
      }
      drv2605.set_waveform(1, Driver::Waveform::END, ec);
      if (ec) {
        logger.error("set waveform failed: {}", ec.message());
        return false;
      }
      drv2605.start(ec);
      if (ec) {
        logger.error("start failed: {}", ec.message());
        return false;
      }
      waveform++;
      if (waveform > (uint8_t)Driver::Waveform::MAX) {
        waveform = 0;
      }
      logger.info("{}", waveform);
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        using namespace std::chrono_literals;
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 1s);
      }
      // we don't want to stop, so return false
      return false;
    };
    auto task = espp::Task::make_unique({.callback = task_fn,
                                         .task_config =
                                             {
                                                 .name = "example",
                                                 .stack_size_bytes{4 * 1024},
                                             },
                                         .log_level = espp::Logger::Verbosity::INFO});
    task->start();
    //! [drv2605 example]
    logger.info("%time (s), waveform");
    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  logger.info("Drv2605 example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
