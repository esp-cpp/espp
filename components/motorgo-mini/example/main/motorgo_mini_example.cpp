#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "butterworth_filter.hpp"
#include "high_resolution_timer.hpp"
#include "motorgo-mini.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "MotorGo Mini Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting");
  //! [motorgo-mini example]
  auto &motorgo_mini = espp::MotorGoMini::get();
  motorgo_mini.set_log_level(espp::Logger::Verbosity::INFO);
  motorgo_mini.init_motor_channel_1();
  motorgo_mini.init_motor_channel_2();
  auto &motor1 = motorgo_mini.motor1();
  auto &motor2 = motorgo_mini.motor2();
  auto &button = motorgo_mini.button();

  static constexpr uint64_t core_update_period_us = 1000;                   // microseconds
  static constexpr float core_update_period = core_update_period_us / 1e6f; // seconds

  // static constexpr auto motion_control_type = espp::detail::MotionControlType::VELOCITY_OPENLOOP;
  // static constexpr auto motion_control_type = espp::detail::MotionControlType::VELOCITY;
  // static const auto motion_control_type = espp::detail::MotionControlType::ANGLE_OPENLOOP;
  static auto motion_control_type = espp::detail::MotionControlType::ANGLE;

  logger.info("Setting motion control type to {}", motion_control_type);
  motor1.set_motion_control_type(motion_control_type);
  motor2.set_motion_control_type(motion_control_type);

  motor1.enable();
  motor2.enable();

  std::atomic<float> target1 = 60.0f;
  std::atomic<float> target2 = 60.0f;
  static bool target_is_angle =
      motion_control_type == espp::detail::MotionControlType::ANGLE ||
      motion_control_type == espp::detail::MotionControlType::ANGLE_OPENLOOP;
  // Function for initializing the target based on the motion control type
  auto initialize_target = [&]() {
    if (target_is_angle) {
      target1 = motor1.get_shaft_angle();
      target2 = motor2.get_shaft_angle();
    } else {
      target1 = 50.0f * espp::RPM_TO_RADS;
      target2 = 50.0f * espp::RPM_TO_RADS;
    }
  };
  // run it once
  initialize_target();

  auto dual_motor_fn = [&]() -> bool {
    motor1.loop_foc();
    motor2.loop_foc();
    motor1.move(target1);
    motor2.move(target2);
    return false; // don't want to stop the task
  };

  auto dual_motor_timer = espp::HighResolutionTimer({.name = "Motor Timer",
                                                     .callback = dual_motor_fn,
                                                     .log_level = espp::Logger::Verbosity::WARN});
  dual_motor_timer.periodic(core_update_period_us);

  static constexpr float sample_freq_hz = 100.0f;
  static constexpr float filter_cutoff_freq_hz = 5.0f;
  static constexpr float normalized_cutoff_frequency =
      2.0f * filter_cutoff_freq_hz / sample_freq_hz;
  static constexpr size_t ORDER = 2;
  // NOTE: using the Df2 since it's hardware accelerated :)
  using Filter = espp::ButterworthFilter<ORDER, espp::BiquadFilterDf2>;
  Filter filter1({.normalized_cutoff_frequency = normalized_cutoff_frequency});
  Filter filter2({.normalized_cutoff_frequency = normalized_cutoff_frequency});

  // if it's a velocity setpoint then target is RPM
  fmt::print("%time(s), "
             "motor 1 target, " // target is either RPM or radians
             "motor 1 angle (radians), "
             "motor 1 speed (rpm), "
             "motor 2 target, " // target is either RPM or radians
             "motor 2 angle (radians), "
             "motor 2 speed (rpm)\n");

  // make the task to periodically poll the encoders and print the state. NOTE:
  // the encoders run their own tasks to maintain state, so we're just polling
  // the current state.
  auto logging_fn = [&](std::mutex &m, std::condition_variable &cv) {
    static auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto seconds = std::chrono::duration<float>(now - start).count();
    auto _target1 = target1.load();
    if (!target_is_angle)
      _target1 *= espp::RADS_TO_RPM;
    auto _target2 = target2.load();
    if (!target_is_angle)
      _target2 *= espp::RADS_TO_RPM;
    auto rpm1 = filter1(motor1.get_shaft_velocity() * espp::RADS_TO_RPM);
    auto rpm2 = filter2(motor2.get_shaft_velocity() * espp::RADS_TO_RPM);
    auto rads1 = motor1.get_shaft_angle();
    auto rads2 = motor2.get_shaft_angle();
    fmt::print("{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}\n", seconds, _target1, rads1,
               rpm1, _target2, rads2, rpm2);
    // NOTE: sleeping in this way allows the sleep to exit early when the
    // task is being stopped / destroyed
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, 10ms);
    }
    // don't want to stop the task
    return false;
  };
  auto logging_task = espp::Task({.callback = logging_fn,
                                  .task_config =
                                      {
                                          .name = "Logging Task",
                                          .stack_size_bytes = 5 * 1024,
                                      },
                                  .log_level = espp::Logger::Verbosity::WARN});
  logging_task.start();

  std::this_thread::sleep_for(1s);
  logger.info("Starting target task");

  enum class IncrementDirection { DOWN = -1, HOLD = 0, UP = 1 };
  static IncrementDirection increment_direction1 = IncrementDirection::UP;
  static IncrementDirection increment_direction2 = IncrementDirection::DOWN;

  auto update_target = [&](auto &target, auto &increment_direction) {
    float max_target = target_is_angle ? (2.0f * M_PI) : (200.0f * espp::RPM_TO_RADS);
    float target_delta =
        target_is_angle ? (M_PI / 4.0f) : (50.0f * espp::RPM_TO_RADS * core_update_period);
    // update target
    if (increment_direction == IncrementDirection::UP) {
      target += target_delta;
      if (target >= max_target) {
        increment_direction = IncrementDirection::DOWN;
      }
    } else if (increment_direction == IncrementDirection::DOWN) {
      target -= target_delta;
      if (target <= -max_target) {
        increment_direction = IncrementDirection::UP;
      }
    }
  };

  // make a task which will update the target (velocity or angle)
  auto target_task_fn = [&](std::mutex &m, std::condition_variable &cv) {
    auto delay = std::chrono::duration<float>(target_is_angle ? 1.0f : core_update_period);
    auto start = std::chrono::high_resolution_clock::now();
    update_target(target1, increment_direction1);
    update_target(target2, increment_direction2);
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

  bool button_state = false;

  while (true) {
    bool new_button_state = button.is_pressed();
    if (new_button_state != button_state) {
      button_state = new_button_state;
      if (button_state) {
        logger.info("Button pressed, changing motion control type");
        // switch between ANGLE and VELOCITY
        if (motion_control_type == espp::detail::MotionControlType::ANGLE ||
            motion_control_type == espp::detail::MotionControlType::ANGLE_OPENLOOP) {
          motion_control_type = espp::detail::MotionControlType::VELOCITY;
          target_is_angle = false;
        } else {
          motion_control_type = espp::detail::MotionControlType::ANGLE;
          target_is_angle = true;
        }
        initialize_target();
        motor1.set_motion_control_type(motion_control_type);
        motor2.set_motion_control_type(motion_control_type);
      } else {
        logger.info("Button released");
      }
    }
    std::this_thread::sleep_for(50ms);
  }
  //! [motorgo-mini example]
}
