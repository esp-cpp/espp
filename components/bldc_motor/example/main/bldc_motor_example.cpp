#include <chrono>
#include <memory>
#include <sdkconfig.h>
#include <vector>

#include "bldc_driver.hpp"
#include "bldc_motor.hpp"
#include "butterworth_filter.hpp"
#include "high_resolution_timer.hpp"
#include "i2c.hpp"
#include "logger.hpp"
#include "mt6701.hpp"
#include "task.hpp"
#include "task_monitor.hpp"

#if CONFIG_EXAMPLE_HARDWARE_MOTORGO_MINI
#include "motorgo-mini.hpp"
#elif CONFIG_EXAMPLE_HARDWARE_MOTORGO_AXIS
#include "motorgo-axis.hpp"
#endif

using namespace std::chrono_literals;

// The MotorGo boards route the magnetic encoder over an SSI bus, while the
// test-stand / custom wiring uses an I2C MT6701.
#if CONFIG_EXAMPLE_HARDWARE_MOTORGO_MINI || CONFIG_EXAMPLE_HARDWARE_MOTORGO_AXIS
using Encoder = espp::Mt6701<espp::Mt6701Interface::SSI>;
#else
using Encoder = espp::Mt6701<>;
#endif
using BldcMotor = espp::BldcMotor<espp::BldcDriver, Encoder>;

// Which MotorGo channel to drive (index 0 == "Motor 1", index 1 == "Motor 2").
#if CONFIG_EXAMPLE_MOTOR_CHANNEL_2
static constexpr size_t example_motor_index = 1;
#else
static constexpr size_t example_motor_index = 0;
#endif

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "BLDC Motor example", .level = espp::Logger::Verbosity::DEBUG});
  constexpr int num_seconds_to_run = 120;
  {
    logger.info("Running BLDC Motor (FOC) example for {} seconds!", num_seconds_to_run);

    static constexpr uint64_t core_update_period_us = 1000;                   // microseconds
    static constexpr float core_update_period = core_update_period_us / 1e6f; // seconds

    // The motor and driver are set up below depending on the selected hardware.
    std::shared_ptr<espp::BldcDriver> driver;
    std::shared_ptr<BldcMotor> motor;
    // Objects which must outlive the motor for the standalone (I2C) wiring.
    std::unique_ptr<espp::I2c> i2c;
    std::shared_ptr<Encoder> standalone_encoder;

    //! [bldc_motor example]
#if CONFIG_EXAMPLE_HARDWARE_MOTORGO_MINI || CONFIG_EXAMPLE_HARDWARE_MOTORGO_AXIS
#if CONFIG_EXAMPLE_HARDWARE_MOTORGO_MINI
    using Board = espp::MotorGoMini;
    logger.info("Using MotorGo Mini, motor channel {}", example_motor_index + 1);
#else
    using Board = espp::MotorGoAxis;
    logger.info("Using MotorGo Axis, motor channel {}", example_motor_index + 1);
#endif
    // Both MotorGo boards expose the same symmetric, index-based API, so the
    // rest of the setup is identical regardless of which board is selected.
    auto &board = Board::get();
    board.set_log_level(espp::Logger::Verbosity::INFO);
    board.initialize_encoders(); // start the encoder update task(s)
    board.initialize_motors();   // create the motor driver(s)
    auto motor_config = board.default_motor_config(example_motor_index);
    // tweak motor_config here if desired (PID gains, current limit, etc.)
    motor = board.initialize_motor(example_motor_index, motor_config);
    driver = board.motor_driver(example_motor_index);
#else
    logger.info("Using test-stand / custom wiring (I2C MT6701 + TMC6300)");
    // make the I2C that we'll use to communicate with the mt6701 (magnetic encoder)
    logger.info("initializing i2c driver...");
    i2c = std::make_unique<espp::I2c>(espp::I2c::Config{
        .port = I2C_NUM_1,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
        .clk_speed = 1 * 1000 * 1000, // MT6701 supports 1 MHz I2C
    });

    // now make the mt6701 which decodes the data
    std::error_code ec;
    auto encoder_device =
        i2c->add_device<uint8_t>({.device_address = Encoder::DEFAULT_ADDRESS,
                                  .timeout_ms = static_cast<int>(i2c->config().timeout_ms),
                                  .scl_speed_hz = i2c->config().clk_speed,
                                  .log_level = espp::Logger::Verbosity::WARN},
                                 ec);
    if (!encoder_device) {
      logger.error("Failed to initialize MT6701 I2C device: {}", ec.message());
      return;
    }
    standalone_encoder = std::make_shared<Encoder>(
        Encoder::Config{.write = espp::make_i2c_addressed_write(encoder_device),
                        .read = espp::make_i2c_addressed_read(encoder_device),
                        .velocity_filter = nullptr, // no filtering
                        .update_period = std::chrono::duration<float>(core_update_period),
                        .log_level = espp::Logger::Verbosity::WARN});

    // now make the bldc driver
    driver = std::make_shared<espp::BldcDriver>(espp::BldcDriver::Config{
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
    motor = std::make_shared<BldcMotor>(BldcMotor::Config{
        // measured by setting it into ANGLE_OPENLOOP and then counting how many
        // spots you feel when rotating it.
        .num_pole_pairs = 7,
        .phase_resistance =
            5.0f, // tested by running velocity_openloop and seeing if the veloicty is ~correct
        .kv_rating =
            320, // tested by running velocity_openloop and seeing if the velocity is ~correct
        .current_limit = 1.0f,        // Amps
        .zero_electric_offset = 0.0f, // set to zero to always calibrate, since this is a test
        .sensor_direction = espp::detail::SensorDirection::UNKNOWN, // set to unknown to always
                                                                    // calibrate, since this is a
                                                                    // test
        .foc_type = espp::detail::FocType::SPACE_VECTOR_PWM,
        .driver = driver,
        .sensor = standalone_encoder,
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
#endif

    if (!motor || !driver) {
      logger.error("Motor / driver were not initialized, cannot run example!");
      return;
    }

    static const auto motion_control_type = espp::detail::MotionControlType::VELOCITY;
    // static const auto motion_control_type = espp::detail::MotionControlType::ANGLE;
    // static const auto motion_control_type = espp::detail::MotionControlType::VELOCITY_OPENLOOP;
    // static const auto motion_control_type = espp::detail::MotionControlType::ANGLE_OPENLOOP;

    // Set the motion control type and create a target for the motor (will be
    // updated in the target update task below)
    motor->set_motion_control_type(motion_control_type);
    std::atomic<float> target = 0;

    // enable the motor
    motor->enable();

    auto motor_task_fn = [&motor, &target]() -> bool {
      if (motion_control_type == espp::detail::MotionControlType::VELOCITY ||
          motion_control_type == espp::detail::MotionControlType::VELOCITY_OPENLOOP) {
        // if it's a velocity setpoint, convert it from RPM to rad/s
        motor->move(target * espp::RPM_TO_RADS);
      } else {
        motor->move(target);
      }
      // command the motor
      motor->loop_foc();
      // don't want to stop the task
      return false;
    };
    auto motor_timer = espp::HighResolutionTimer({.name = "Motor Timer",
                                                  .callback = motor_task_fn,
                                                  .log_level = espp::Logger::Verbosity::WARN});
    motor_timer.periodic(core_update_period_us);
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
      target = motor->get_shaft_angle();
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
        .callback = target_task_fn,
        .task_config = {.name = "Target Task"},
    });
    target_task.start();

    static constexpr float sample_freq_hz = 100.0f;
    static constexpr float filter_cutoff_freq_hz = 5.0f;
    static constexpr float normalized_cutoff_frequency =
        2.0f * filter_cutoff_freq_hz / sample_freq_hz;
    static constexpr size_t ORDER = 2;
    // NOTE: using the Df2 since it's hardware accelerated :)
    using Filter = espp::ButterworthFilter<ORDER, espp::BiquadFilterDf2>;
    Filter filter({.normalized_cutoff_frequency = normalized_cutoff_frequency});

    // and finally, make the task to periodically poll the mt6701 and print the
    // state. NOTE: the Mt6701 runs its own task to maintain state, so we're
    // just polling the current state.
    auto task_fn = [&](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds = std::chrono::duration<float>(now - start).count();
      auto radians = motor->get_shaft_angle();
      auto degrees = radians * 180.0f / M_PI;
      auto rpm = filter(motor->get_shaft_velocity() * espp::RADS_TO_RPM);
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
    auto task = espp::Task({.callback = task_fn,
                            .task_config =
                                {
                                    .name = "Logging Task",
                                    .stack_size_bytes = 5 * 1024,
                                },
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

  logger.info("BLDC Motor (FOC) example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
