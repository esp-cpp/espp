#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "butterworth_filter.hpp"
#include "high_resolution_timer.hpp"
#include "motorgo-mini.hpp"
#include "task.hpp"
#include "task_monitor.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "MotorGo Mini Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting");
  //! [motorgo-mini example]
  espp::MotorGoMini motorgo_mini(espp::Logger::Verbosity::INFO);
  auto motor1 = motorgo_mini.motor1();
  auto motor2 = motorgo_mini.motor2();
  auto &button = motorgo_mini.button();

  static constexpr uint64_t core_update_period_us = 1000;                   // microseconds
  static constexpr float core_update_period = core_update_period_us / 1e6f; // seconds

  // static constexpr auto motion_control_type = espp::detail::MotionControlType::VELOCITY_OPENLOOP;
  // static constexpr auto motion_control_type = espp::detail::MotionControlType::VELOCITY;
  // static const auto motion_control_type = espp::detail::MotionControlType::ANGLE_OPENLOOP;
  static const auto motion_control_type = espp::detail::MotionControlType::ANGLE;

  logger.info("Setting motion control type to {}", motion_control_type);
  motor1->set_motion_control_type(motion_control_type);
  motor2->set_motion_control_type(motion_control_type);

  motor1->enable();
  motor2->enable();

  std::atomic<float> target1 = 60.0f;
  std::atomic<float> target2 = 60.0f;

  auto motor_task_fn = [&](auto &motor, auto &target) -> bool {
    if constexpr (motion_control_type == espp::detail::MotionControlType::VELOCITY ||
                  motion_control_type == espp::detail::MotionControlType::VELOCITY_OPENLOOP) {
      // if it's a velocity setpoint, convert it from RPM to rad/s
      motor->move(target * espp::RPM_TO_RADS);
    } else {
      // it's a position setpoint, so just set the target
      motor->move(target);
    }
    // command the motor
    motor->loop_foc();
    return false; // don't want to stop the task
  };
  auto motor1_fn = std::bind(motor_task_fn, std::ref(motor1), std::ref(target1));
  auto motor2_fn = std::bind(motor_task_fn, std::ref(motor2), std::ref(target2));

  auto dual_motor_fn = [&]() -> bool {
    motor1_fn();
    motor2_fn();
    return false; // don't want to stop the task
  };

  // auto motor1_timer = espp::HighResolutionTimer(
  //     {.name = "Motor 1 Timer", .callback = motor1_fn, .log_level =
  //     espp::Logger::Verbosity::WARN});
  // motor1_timer.periodic(core_update_period_us);

  // auto motor2_timer = espp::HighResolutionTimer(
  //     {.name = "Motor 2 Timer", .callback = motor2_fn, .log_level =
  //     espp::Logger::Verbosity::WARN});
  // motor2_timer.periodic(core_update_period_us);

  auto dual_motor_timer = espp::HighResolutionTimer({.name = "Motor Timer",
                                                     .callback = dual_motor_fn,
                                                     .log_level = espp::Logger::Verbosity::WARN});
  // NOTE: we'll start the timer when the button is pressed
  // dual_motor_timer.periodic(core_update_period_us);

  // Function for initializing the target based on the motion control type
  auto initialize_target = [&]() {
    switch (motion_control_type) {
    case espp::detail::MotionControlType::VELOCITY:
    case espp::detail::MotionControlType::VELOCITY_OPENLOOP:
      target1 = 50.0f;
      target2 = 50.0f;
      break;
    case espp::detail::MotionControlType::ANGLE:
    case espp::detail::MotionControlType::ANGLE_OPENLOOP:
      target1 = motor1->get_shaft_angle();
      target2 = motor2->get_shaft_angle();
      break;
    default:
      break;
    }
  };

  static constexpr float filter_cutoff_hz = 4.0f;
  // we're running this in the logging task, which is 0.01s (10ms). We'll use a
  // filter only on the velocity.
  espp::ButterworthFilter<2, espp::BiquadFilterDf2> filter1(
      {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * 0.01f});
  espp::ButterworthFilter<2, espp::BiquadFilterDf2> filter2(
      {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * 0.01f});

  // make the task to periodically poll the encoders and print the state. NOTE:
  // the encoders run their own tasks to maintain state, so we're just polling
  // the current state.
  auto logging_fn = [&](std::mutex &m, std::condition_variable &cv) {
    static auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto seconds = std::chrono::duration<float>(now - start).count();
    auto _target1 = target1.load();
    auto _target2 = target2.load();
    if constexpr (motion_control_type == espp::detail::MotionControlType::VELOCITY ||
                  motion_control_type == espp::detail::MotionControlType::VELOCITY_OPENLOOP) {
      auto rpm1 = filter1.update(motor1->get_shaft_velocity() * espp::RADS_TO_RPM);
      auto rpm2 = filter2.update(motor2->get_shaft_velocity() * espp::RADS_TO_RPM);
      fmt::print("{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}\n", seconds, _target1, rpm1, _target2,
                 rpm2);
    } else {
      auto rads1 = motor1->get_shaft_angle();
      auto rads2 = motor2->get_shaft_angle();
      fmt::print("{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}\n", seconds, _target1, rads1, _target2,
                 rads2);
    }
    // NOTE: sleeping in this way allows the sleep to exit early when the
    // task is being stopped / destroyed
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, 10ms);
    }
    // don't want to stop the task
    return false;
  };
  auto logging_task = espp::Task({.name = "Logging Task",
                                  .callback = logging_fn,
                                  .stack_size_bytes = 5 * 1024,
                                  .log_level = espp::Logger::Verbosity::WARN});
  if constexpr (motion_control_type == espp::detail::MotionControlType::VELOCITY ||
                motion_control_type == espp::detail::MotionControlType::VELOCITY_OPENLOOP) {
    // if it's a velocity setpoint then target is RPM
    fmt::print(
        "%time(s), target velocity 1 (rpm), motor 1 actual speed (rpm), motor 2 actual speed "
        "(rpm), target velocity 2 (rpm)\n");
  } else {
    // if it's an angle setpoint then target is angle (radians)
    fmt::print("%time(s), target angle 1 (radians), motor 1 actual angle (radians), motor 2 actual "
               "angle (radians), target angle 2 (radians)\n");
  }
  logging_task.start();

  std::this_thread::sleep_for(1s);
  logger.info("Starting target task");

  enum class IncrementDirection { DOWN = -1, HOLD = 0, UP = 1 };
  static IncrementDirection increment_direction1 = IncrementDirection::UP;
  static IncrementDirection increment_direction2 = IncrementDirection::DOWN;
  static const bool is_angle =
      motion_control_type == espp::detail::MotionControlType::ANGLE ||
      motion_control_type == espp::detail::MotionControlType::ANGLE_OPENLOOP;
  static const float max_target = is_angle ? (2.0f * M_PI) : 200.0f;
  static const float target_delta = is_angle ? (M_PI / 4.0f) : (50.0f * core_update_period);

  auto update_target = [&](auto &target, auto &increment_direction) {
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
    static auto delay = std::chrono::duration<float>(is_angle ? 1.0f : core_update_period);
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
      .name = "Target Task",
      .callback = target_task_fn,
  });
  target_task.start();

  bool button_state = false;

  //! [motorgo-mini example]
  while (true) {
    // fmt::print("{}", espp::TaskMonitor::get_latest_info_table());
    bool new_button_state = button.is_pressed();
    if (new_button_state != button_state) {
      button_state = new_button_state;
      if (button_state) {
        logger.info("Button pressed, starting motors");
        initialize_target();
        dual_motor_timer.periodic(core_update_period_us);
      } else {
        logger.info("Button released, stopping motors");
        dual_motor_timer.stop();
      }
    }
    std::this_thread::sleep_for(50ms);
  }
}
