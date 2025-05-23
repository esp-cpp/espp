#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "bldc_driver.hpp"
#include "bldc_haptics.hpp"
#include "bldc_motor.hpp"
#include "i2c.hpp"
#include "mt6701.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "BLDC Haptics Example", .level = espp::Logger::Verbosity::DEBUG});
  constexpr int num_seconds_to_run = 20;
  {
    logger.info("Running BLDC Haptics example for {} seconds!", num_seconds_to_run);

    // make the I2C that we'll use to communicate with the mt6701 (magnetic encoder)
    logger.info("initializing i2c driver...");
    espp::I2c i2c({
        .port = I2C_NUM_1,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
        .clk_speed = 1 * 1000 * 1000, // MT6701 supports 1 MHz I2C
    });

    static constexpr float core_update_period = 0.001f; // seconds

    // now make the mt6701 which decodes the data
    using Encoder = espp::Mt6701<>;
    std::shared_ptr<Encoder> mt6701 = std::make_shared<Encoder>(
        Encoder::Config{.write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1,
                                           std::placeholders::_2, std::placeholders::_3),
                        .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1,
                                          std::placeholders::_2, std::placeholders::_3),
                        .velocity_filter = nullptr, // no filtering
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
            .log_level = espp::Logger::Verbosity::DEBUG});

    // now make the bldc motor
    using BldcMotor = espp::BldcMotor<espp::BldcDriver, Encoder>;
    auto motor = std::make_shared<BldcMotor>(BldcMotor::Config{
        // measured by setting it into ANGLE_OPENLOOP and then counting how many
        // spots you feel when rotating it.
        .num_pole_pairs = 7,
        .phase_resistance =
            5.0f, // tested by running velocity_openloop and seeing if the veloicty is ~correct
        .kv_rating =
            320, // tested by running velocity_openloop and seeing if the velocity is ~correct
        .current_limit = 1.0f,        // Amps
        .zero_electric_offset = 0.0f, // set to zero to always calibrate, since this is a test
        .sensor_direction =
            espp::detail::SensorDirection::UNKNOWN, // set to unknown to always calibrate, since
                                                    // this is a test
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
        .log_level = espp::Logger::Verbosity::DEBUG});

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
                                     .kp_factor = 2,
                                     .kd_factor_min = 0.01,
                                     .kd_factor_max = 0.04,
                                     .log_level = espp::Logger::Verbosity::INFO});

    // auto detent_config = espp::detail::UNBOUNDED_NO_DETENTS;
    // auto detent_config = espp::detail::BOUNDED_NO_DETENTS;
    // auto detent_config = espp::detail::MULTI_REV_NO_DETENTS;
    // auto detent_config = espp::detail::ON_OFF_STRONG_DETENTS;
    auto detent_config = espp::detail::COARSE_VALUES_STRONG_DETENTS;
    // auto detent_config = espp::detail::FINE_VALUES_NO_DETENTS;
    // auto detent_config = espp::detail::FINE_VALUES_WITH_DETENTS;
    // auto detent_config = espp::detail::MAGNETIC_DETENTS;
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
      logger.info("Playing haptic click!");
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

  logger.info("BLDC Haptics example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
