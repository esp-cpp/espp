#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "motorgo-mini.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "MotorGo Mini Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting");
  //! [motorgo-mini example]
  espp::MotorGoMini motorgo_mini;
  auto &motor1 = motorgo_mini.motor1();
  auto &motor2 = motorgo_mini.motor2();

  static constexpr auto motion_control_type = espp::detail::MotionControlType::VELOCITY;
  motor1.set_motion_control_type(motion_control_type);
  motor2.set_motion_control_type(motion_control_type);

  motor1.enable();
  motor2.enable();

  std::atomic<float> target = 60.0f;

  auto motor_task_fn = [&motor1, &motor2, &target](std::mutex &m, std::condition_variable &cv) {
    static constexpr float core_update_period = 0.001f; // seconds
    static auto delay = std::chrono::duration<float>(core_update_period);
    auto start = std::chrono::high_resolution_clock::now();
    if constexpr (motion_control_type == espp::detail::MotionControlType::VELOCITY ||
                  motion_control_type == espp::detail::MotionControlType::VELOCITY_OPENLOOP) {
      // if it's a velocity setpoint, convert it from RPM to rad/s
      motor1.move(target * espp::RPM_TO_RADS);
      motor2.move(target * espp::RPM_TO_RADS);
    } else {
      // it's a position setpoint, so just set the target
      motor1.move(target);
      motor2.move(target);
    }
    // command the motor
    motor1.loop_foc();
    motor2.loop_foc();
    // NOTE: sleeping in this way allows the sleep to exit early when the
    // task is being stopped / destroyed
    std::unique_lock<std::mutex> lk(m);
    cv.wait_until(lk, start + delay);
    return false; // don't want to stop the task
  };
  auto motor_task = espp::Task({.name = "Motor Task",
                                .callback = motor_task_fn,
                                .stack_size_bytes = 5 * 1024,
                                .priority = 20,
                                .core_id = 1,
                                .log_level = espp::Logger::Verbosity::WARN});
  motor_task.start();

  // make the task to periodically poll the encoders and print the state. NOTE:
  // the encoders run their own tasks to maintain state, so we're just polling
  // the current state.
  auto task_fn = [&motor1, &motor2, &target](std::mutex &m, std::condition_variable &cv) {
    static auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto seconds = std::chrono::duration<float>(now - start).count();
    auto _target = target.load();
    if constexpr (motion_control_type == espp::detail::MotionControlType::VELOCITY ||
                  motion_control_type == espp::detail::MotionControlType::VELOCITY_OPENLOOP) {
      auto rpm1 = motor1.get_shaft_velocity() * espp::RADS_TO_RPM;
      auto rpm2 = motor2.get_shaft_velocity() * espp::RADS_TO_RPM;
      fmt::print("{:.3f}, {:.3f}, {:.3f}, {:.3f}\n", seconds, _target, rpm1, rpm2);
    } else {
      auto rads1 = motor1.get_shaft_angle();
      auto rads2 = motor2.get_shaft_angle();
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

  //! [motorgo-mini example]
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
