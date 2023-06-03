#include <chrono>
#include <vector>

#include "driver/i2c.h"

#include "bldc_driver.hpp"
#include "bldc_motor.hpp"
#include "butterworth_filter.hpp"
#include "logger.hpp"
#include "lowpass_filter.hpp"
#include "mt6701.hpp"
#include "task.hpp"
#include "task_monitor.hpp"

using namespace std::chrono_literals;

// pins for the bldc motor test stand with the TinyS3
static constexpr auto I2C_NUM = (I2C_NUM_1);
static constexpr auto I2C_SCL_IO = (GPIO_NUM_9);
static constexpr auto I2C_SDA_IO = (GPIO_NUM_8);
static constexpr int I2C_FREQ_HZ = (400 * 1000);
static constexpr int I2C_TIMEOUT_MS = (10);

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "BLDC Motor example", .level = espp::Logger::Verbosity::DEBUG});
  constexpr int num_seconds_to_run = 120;
  {
    logger.info("Running BLDC Motor (FOC) example for {} seconds!", num_seconds_to_run);

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
    auto mt6701_write = [](uint8_t dev_addr, uint8_t *data, size_t data_len) {
      i2c_master_write_to_device(I2C_NUM, dev_addr, data, data_len,
                                 I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    };

    auto mt6701_read = [](uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t data_len) {
      i2c_master_write_read_device(I2C_NUM, dev_addr, &reg_addr, 1, data, data_len,
                                   I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    };

    // make the velocity filter
    static constexpr float core_update_period = 0.001f; // seconds
    static constexpr float filter_cutoff_hz = 4.0f;
    espp::ButterworthFilter<2, espp::BiquadFilterDf2> bwfilter(
        {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * core_update_period});
    espp::LowpassFilter lpfilter(
        {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * core_update_period,
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

    //! [bldc_motor example]
    // now make the mt6701 which decodes the data
    std::shared_ptr<espp::Mt6701> mt6701 = std::make_shared<espp::Mt6701>(
        espp::Mt6701::Config{.write = mt6701_write,
                             .read = mt6701_read,
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

    static const auto motion_control_type = espp::detail::MotionControlType::VELOCITY;
    // static const auto motion_control_type = espp::detail::MotionControlType::ANGLE;
    // static const auto motion_control_type = espp::detail::MotionControlType::VELOCITY_OPENLOOP;
    // static const auto motion_control_type = espp::detail::MotionControlType::ANGLE_OPENLOOP;

    // Set the motion control type and create a target for the motor (will be
    // updated in the target update task below)
    motor.set_motion_control_type(motion_control_type);
    std::atomic<float> target = 0;

    auto motor_task_fn = [&motor, &target](std::mutex &m, std::condition_variable &cv) {
      static auto delay = std::chrono::duration<float>(core_update_period);
      auto start = std::chrono::high_resolution_clock::now();
      if (motion_control_type == espp::detail::MotionControlType::VELOCITY ||
          motion_control_type == espp::detail::MotionControlType::VELOCITY_OPENLOOP) {
        // if it's a velocity setpoint, convert it from RPM to rad/s
        motor.move(target * espp::RPM_TO_RADS);
      } else {
        motor.move(target);
      }
      // command the motor
      motor.loop_foc();
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_until(lk, start + delay);
      }
      // don't want to stop the task
      return false;
    };
    auto motor_task = espp::Task({.name = "Motor Task",
                                  .callback = motor_task_fn,
                                  .stack_size_bytes = 5 * 1024,
                                  .priority = 20,
                                  .core_id = 1,
                                  .log_level = espp::Logger::Verbosity::WARN});
    motor_task.start();
    //! [bldc_motor example]

    // Configure the target
    enum class IncrementDirection { DOWN = -1, HOLD = 0, UP = 1 };
    static IncrementDirection increment_direction = IncrementDirection::UP;
    static const bool is_angle =
        motion_control_type == espp::detail::MotionControlType::ANGLE ||
        motion_control_type == espp::detail::MotionControlType::ANGLE_OPENLOOP;
    static const float max_target = is_angle ? (2.0f * M_PI) : 200.0f;
    static const float target_delta = is_angle ? (M_PI / 4.0f) : (50.0f * core_update_period);
    switch (motion_control_type) {
    case espp::detail::MotionControlType::VELOCITY:
    case espp::detail::MotionControlType::VELOCITY_OPENLOOP:
      target = 50.0f;
      break;
    case espp::detail::MotionControlType::ANGLE:
    case espp::detail::MotionControlType::ANGLE_OPENLOOP:
      target = M_PI; // 180 degrees (whereever that is...)
      break;
    default:
      break;
    }

    // make a task which will update the target (velocity or angle)
    auto target_task_fn = [&target](std::mutex &m, std::condition_variable &cv) {
      static auto delay = std::chrono::duration<float>(is_angle ? 1.0f : core_update_period);
      auto start = std::chrono::high_resolution_clock::now();
      // update target
      if (increment_direction == IncrementDirection::UP) {
        target += target_delta;
        if (target > max_target) {
          increment_direction = IncrementDirection::DOWN;
          target -= target_delta;
        }
      } else if (increment_direction == IncrementDirection::DOWN) {
        target -= target_delta;
        if (target < -max_target) {
          increment_direction = IncrementDirection::UP;
          target += target_delta;
        }
      }
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_until(lk, start + delay);
      }
      // don't want to stop the task
      return false;
    };
    auto target_task = espp::Task({
        .name = "Target Task",
        .callback = target_task_fn,
    });
    target_task.start();

    // and finally, make the task to periodically poll the mt6701 and print the
    // state. NOTE: the Mt6701 runs its own task to maintain state, so we're
    // just polling the current state.
    auto task_fn = [&motor, &target](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds = std::chrono::duration<float>(now - start).count();
      auto radians = motor.get_shaft_angle();
      auto degrees = radians * 180.0f / M_PI;
      auto rpm = motor.get_shaft_velocity() * espp::RADS_TO_RPM;
      auto _target = target.load();
      fmt::print("{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}\n", seconds, radians, degrees, _target,
                 rpm);
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 10ms);
      }
      // don't want to stop the task
      return false;
    };
    auto task = espp::Task({.name = "Logging Task",
                            .callback = task_fn,
                            .stack_size_bytes = 5 * 1024,
                            .log_level = espp::Logger::Verbosity::WARN});
    if (motion_control_type == espp::detail::MotionControlType::VELOCITY ||
        motion_control_type == espp::detail::MotionControlType::VELOCITY_OPENLOOP) {
      // if it's a velocity setpoint then target is RPM
      fmt::print("%time(s), radians, degrees, target velocity (rpm), actual speed (rpm)\n");
    } else {
      // if it's an angle setpoint then target is angle (radians)
      fmt::print("%time(s), radians, degrees, target angle (radians), actual speed (rpm)\n");
    }
    task.start();

    static auto start = std::chrono::high_resolution_clock::now();
    while (true) {
      // check if the driver is faulted
      if (driver->is_faulted()) {
        logger.error("Driver faulted!");
        break;
      }
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds = std::chrono::duration<float>(now - start).count();
      if (seconds > num_seconds_to_run) {
        logger.info("Test time passed, stopping...");
        break;
      }
      std::this_thread::sleep_for(500ms);
    }
  }
  // now clean up the i2c driver
  i2c_driver_delete(I2C_NUM);

  logger.info("BLDC Motor (FOC) example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
