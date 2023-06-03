#include <chrono>
#include <vector>

#include "driver/i2c.h"

#include "bldc_driver.hpp"
#include "bldc_haptics.hpp"
#include "bldc_motor.hpp"
#include "butterworth_filter.hpp"
#include "lowpass_filter.hpp"
#include "mt6701.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

// pins for the bldc motor test stand with the TinyS3
static constexpr auto I2C_NUM = (I2C_NUM_1);
static constexpr auto I2C_SCL_IO = (GPIO_NUM_9);
static constexpr auto I2C_SDA_IO = (GPIO_NUM_8);
static constexpr int I2C_FREQ_HZ = (400 * 1000);
static constexpr int I2C_TIMEOUT_MS = (10);

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "BLDC Haptics Example", .level = espp::Logger::Verbosity::DEBUG});
  constexpr int num_seconds_to_run = 20;
  {
    logger.info("Running BLDC Haptics example for {} seconds!", num_seconds_to_run);

    // make the I2C that we'll use to communicate with the mt6701 (magnetic encoder)
    i2c_config_t i2c_cfg;
    logger.info("initializing i2c driver...");
    memset(&i2c_cfg, 0, sizeof(i2c_cfg));
    i2c_cfg.sda_io_num = I2C_SDA_IO;
    i2c_cfg.scl_io_num = I2C_SCL_IO;
    i2c_cfg.mode = I2C_MODE_MASTER;
    i2c_cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_cfg.master.clk_speed = I2C_FREQ_HZ;
    auto err = i2c_param_config(I2C_NUM, &i2c_cfg);
    if (err != ESP_OK)
      logger.error("config i2c failed");
    err = i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK)
      logger.error("install i2c driver failed");
    // make some lambda functions we'll use to read/write to the mt6701
    auto i2c_write = [](uint8_t dev_addr, uint8_t *data, size_t len) {
      i2c_master_write_to_device(I2C_NUM, dev_addr, data, len, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    };

    auto i2c_read = [](uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
      i2c_master_write_read_device(I2C_NUM, dev_addr, &reg_addr, 1, data, len,
                                   I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    };

    // make the velocity filter
    static constexpr float core_update_period = 0.001f; // seconds
    static constexpr float filter_cutoff_hz = 4.0f;
    espp::ButterworthFilter<2, espp::BiquadFilterDf2> bwfilter({
        .normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * 0.01 // core_update_period
    });
    espp::LowpassFilter lpfilter(
        {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * 0.01, // core_update_period,
         .q_factor = 1.0f});
    auto filter_fn = [&bwfilter, &lpfilter](float raw) -> float {
      // return bwfilter.update(raw);
      // return lpfilter.update(raw);

      // NOTE: right now there seems to be something wrong with the filter
      //       configuration, so we don't filter at all. Either 1) the filtering
      //       is not actually removing the noise we want, 2) it is adding too
      //       much delay for the PID to compensate for, or 3) there is a bug in
      //       the update function which doesn't take previous state into
      //       account?
      return raw;
    };

    // now make the mt6701 which decodes the data
    std::shared_ptr<espp::Mt6701> mt6701 = std::make_shared<espp::Mt6701>(
        espp::Mt6701::Config{.write = i2c_write,
                             .read = i2c_read,
                             .velocity_filter = filter_fn,
                             .update_period = std::chrono::duration<float>(core_update_period),
                             .log_level = espp::Logger::Verbosity::WARN});

    // now make the bldc driver
    std::shared_ptr<espp::BldcDriver> driver =
        std::make_shared<espp::BldcDriver>(espp::BldcDriver::Config{
            // this pinout is configured for the TinyS3 connected to the
            // TMC6300-BOB in the BLDC Motor Test Stand
            .gpio_a_h = 1,
            .gpio_a_l = 2,
            .gpio_b_h = 3,
            .gpio_b_l = 4,
            .gpio_c_h = 5,
            .gpio_c_l = 21,
            .gpio_enable = 34, // connected to the VIO/~Stdby pin of TMC6300-BOB
            .gpio_fault = 36,  // connected to the nFAULT pin of TMC6300-BOB
            .power_supply_voltage = 5.0f,
            .limit_voltage = 5.0f,
            .log_level = espp::Logger::Verbosity::WARN});

    // now make the bldc motor
    using BldcMotor = espp::BldcMotor<espp::BldcDriver, espp::Mt6701>;
    auto motor = BldcMotor(BldcMotor::Config{
        // measured by setting it into ANGLE_OPENLOOP and then counting how many
        // spots you feel when rotating it.
        .num_pole_pairs = 7,
        .phase_resistance =
            5.0f, // tested by running velocity_openloop and seeing if the veloicty is ~correct
        .kv_rating =
            320, // tested by running velocity_openloop and seeing if the velocity is ~correct
        .current_limit = 1.0f,             // Amps
        .zero_electric_offset = 2.3914752, // gotten from previously running without providing this
                                           // and it will be logged.
        .sensor_direction = espp::detail::SensorDirection::COUNTER_CLOCKWISE,
        .foc_type = espp::detail::FocType::SPACE_VECTOR_PWM,
        .driver = driver,
        .sensor = mt6701,
        .velocity_pid_config =
            {
                .kp = 0.010f,
                .ki = 1.000f,
                .kd = 0.000f,
                .integrator_min = -1.0f, // same scale as output_min (so same scale as current)
                .integrator_max = 1.0f,  // same scale as output_max (so same scale as current)
                .output_min = -1.0, // velocity pid works on current (if we have phase resistance)
                .output_max = 1.0,  // velocity pid works on current (if we have phase resistance)
            },
        .angle_pid_config =
            {
                .kp = 7.000f,
                .ki = 0.300f,
                .kd = 0.010f,
                .integrator_min = -10.0f, // same scale as output_min (so same scale as velocity)
                .integrator_max = 10.0f,  // same scale as output_max (so same scale as velocity)
                .output_min = -20.0,      // angle pid works on velocity (rad/s)
                .output_max = 20.0,       // angle pid works on velocity (rad/s)
            },
        .log_level = espp::Logger::Verbosity::INFO});

    auto print_detent_config = [&logger](const auto &detent_config) {
      if (detent_config == espp::detail::UNBOUNDED_NO_DETENTS) {
        logger.info("Setting detent config to UNBOUNDED_NO_DETENTS");
      }
      if (detent_config == espp::detail::BOUNDED_NO_DETENTS) {
        logger.info("Setting detent config to BOUNDED_NO_DETENTS");
      }
      if (detent_config == espp::detail::MULTI_REV_NO_DETENTS) {
        logger.info("Setting detent config to MULTI_REV_NO_DETENTS");
      }
      if (detent_config == espp::detail::ON_OFF_STRONG_DETENTS) {
        logger.info("Setting detent config to ON_OFF_STRONG_DETENTS");
      }
      if (detent_config == espp::detail::COARSE_VALUES_STRONG_DETENTS) {
        logger.info("Setting detent config to COARSE_VALUES_STRONG_DETENTS");
      }
      if (detent_config == espp::detail::FINE_VALUES_NO_DETENTS) {
        logger.info("Setting detent config to FINE_VALUES_NO_DETENTS");
      }
      if (detent_config == espp::detail::FINE_VALUES_WITH_DETENTS) {
        logger.info("Setting detent config to FINE_VALUES_WITH_DETENTS");
      }
      if (detent_config == espp::detail::MAGNETIC_DETENTS) {
        logger.info("Setting detent config to MAGNETIC_DETENTS");
      }
      if (detent_config == espp::detail::RETURN_TO_CENTER_WITH_DETENTS) {
        logger.info("Setting detent config to RETURN_TO_CENTER_WITH_DETENTS");
      }
    };

    //! [bldc_haptics_example_1]
    using BldcHaptics = espp::BldcHaptics<BldcMotor>;

    auto haptic_motor = BldcHaptics({.motor = motor,
                                     .kp_factor = 1,
                                     .kd_factor_min = 0.01,
                                     .kd_factor_max = 0.04,
                                     .log_level = espp::Logger::Verbosity::INFO});

    // auto detent_config = espp::detail::UNBOUNDED_NO_DETENTS;
    // auto detent_config = espp::detail::BOUNDED_NO_DETENTS;
    // auto detent_config = espp::detail::MULTI_REV_NO_DETENTS;
    // auto detent_config = espp::detail::ON_OFF_STRONG_DETENTS;
    // auto detent_config = espp::detail::COARSE_VALUES_STRONG_DETENTS;
    // auto detent_config = espp::detail::FINE_VALUES_NO_DETENTS;
    // auto detent_config = espp::detail::FINE_VALUES_WITH_DETENTS;
    auto detent_config = espp::detail::MAGNETIC_DETENTS;
    // auto detent_config = espp::detail::RETURN_TO_CENTER_WITH_DETENTS;

    logger.info("{}", detent_config);

    haptic_motor.update_detent_config(detent_config);
    // this will start the haptic motor thread which will run in the background.
    // If we want to change the detent config we can call update_detent_config()
    // and it will update the detent config in the background thread.
    haptic_motor.start();
    //! [bldc_haptics_example_1]
    print_detent_config(detent_config);

    static auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto seconds = std::chrono::duration<float>(now - start).count();
    while (seconds < num_seconds_to_run) {
      now = std::chrono::high_resolution_clock::now();
      seconds = std::chrono::duration<float>(now - start).count();
      std::this_thread::sleep_for(500ms);
      if (driver->is_faulted()) {
        logger.error("Driver is faulted, cannot continue haptics");
        break;
      }
    }

    // test the haptic buzz / click
    if (!driver->is_faulted()) {
      logger.info("Playing haptic buzz for 1 second");
      //! [bldc_haptics_example_2]
      haptic_motor.play_haptic(espp::detail::HapticConfig{
          .strength = 5.0f,
          .frequency = 200.0f, // Hz, NOTE: frequency is unused for now
          .duration = 1s       // NOTE: duration is unused for now
      });
      //! [bldc_haptics_example_2]
    }

    driver->disable();
  }
  // now clean up the i2c driver
  i2c_driver_delete(I2C_NUM);

  logger.info("BLDC Haptics example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
