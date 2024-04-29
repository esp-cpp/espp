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
  espp::MotorGoMini motorgo_mini;
  auto encoder1 = motorgo_mini.encoder1();
  auto encoder2 = motorgo_mini.encoder2();
  auto motor1 = motorgo_mini.motor1();
  auto motor2 = motorgo_mini.motor2();

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

  std::atomic<float> target = 60.0f;

  auto motor_task_fn = [&]() -> bool {
    if constexpr (motion_control_type == espp::detail::MotionControlType::VELOCITY ||
                  motion_control_type == espp::detail::MotionControlType::VELOCITY_OPENLOOP) {
      // if it's a velocity setpoint, convert it from RPM to rad/s
      motor1->move(target * espp::RPM_TO_RADS);
      // motor2->move(target * espp::RPM_TO_RADS);
    } else {
      // it's a position setpoint, so just set the target
      motor1->move(target);
      // motor2->move(target);
    }
    // command the motor
    motor1->loop_foc();
    // motor2->loop_foc();
    return false; // don't want to stop the task
  };
  auto motor_timer = espp::HighResolutionTimer({.name = "Motor Timer",
                                                .callback = motor_task_fn,
                                                .log_level = espp::Logger::Verbosity::WARN});
  motor_timer.periodic(core_update_period_us);

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
    target = motor1->get_shaft_angle();
    break;
  default:
    break;
  }

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
  auto task_fn = [&](std::mutex &m, std::condition_variable &cv) {
    static auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto seconds = std::chrono::duration<float>(now - start).count();
    auto _target = target.load();
    if constexpr (motion_control_type == espp::detail::MotionControlType::VELOCITY ||
                  motion_control_type == espp::detail::MotionControlType::VELOCITY_OPENLOOP) {
      auto rpm1 = filter1.update(motor1->get_shaft_velocity() * espp::RADS_TO_RPM);
      auto rpm2 = filter2.update(motor2->get_shaft_velocity() * espp::RADS_TO_RPM);
      fmt::print("{:.3f}, {:.3f}, {:.3f}, {:.3f}\n", seconds, _target, rpm1, rpm2);
    } else {
      auto rads1 = motor1->get_shaft_angle();
      auto rads2 = motor2->get_shaft_angle();
      fmt::print("{:.3f}, {:.3f}, {:.3f}, {:.3f}\n", seconds, _target, rads1, rads2);
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
  auto task = espp::Task({.name = "Logging Task",
                          .callback = task_fn,
                          .stack_size_bytes = 5 * 1024,
                          .log_level = espp::Logger::Verbosity::WARN});
  if constexpr (motion_control_type == espp::detail::MotionControlType::VELOCITY ||
                motion_control_type == espp::detail::MotionControlType::VELOCITY_OPENLOOP) {
    // if it's a velocity setpoint then target is RPM
    fmt::print("%time(s), target velocity (rpm), motor 1 actual speed (rpm), motor 2 actual speed "
               "(rpm)\n");
  } else {
    // if it's an angle setpoint then target is angle (radians)
    fmt::print("%time(s), target angle (radians), motor 1 actual angle (radians), motor 2 actual "
               "angle (radians)\n");
  }
  task.start();

  std::this_thread::sleep_for(1s);
  logger.info("Starting target task");

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

  //! [motorgo-mini example]
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
