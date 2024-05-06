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
  auto encoder1 = motorgo_mini.encoder1();
  auto encoder2 = motorgo_mini.encoder2();
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

  std::atomic<float> target1 = 60.0f;
  std::atomic<float> target2 = 60.0f;

  auto dual_motor_fn = [&]() -> bool {
    motor1->move(target1);
    motor2->move(target2);
    motor1->loop_foc();
    motor2->loop_foc();
    return false; // don't want to stop the task
  };

  auto dual_motor_timer = espp::HighResolutionTimer({.name = "Motor Timer",
                                                     .callback = dual_motor_fn,
                                                     .log_level = espp::Logger::Verbosity::WARN});
  // NOTE: we'll start the timer when the button is pressed

  // Function for initializing the target based on the motion control type
  auto initialize_target = [&]() {
    switch (motion_control_type) {
    case espp::detail::MotionControlType::VELOCITY:
    case espp::detail::MotionControlType::VELOCITY_OPENLOOP:
      target1 = 50.0f * espp::RPM_TO_RADS;
      target2 = 50.0f * espp::RPM_TO_RADS;
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

  static constexpr auto target_text =
      motion_control_type == espp::detail::MotionControlType::VELOCITY ||
              motion_control_type == espp::detail::MotionControlType::VELOCITY_OPENLOOP
          ? "speed (rpm)"
          : "angle (radians)";

  // if it's a velocity setpoint then target is RPM
  fmt::print("%time(s), "
             "motor 1 target {0}, "
             "motor 1 angle (radians), "
             "motor 1 speed (rpm), "
             "motor 2 target {0}, "
             "motor 2 angle (radians), "
             "motor 2 speed (rpm)\n",
             target_text);

  // make the task to periodically poll the encoders and print the state. NOTE:
  // the encoders run their own tasks to maintain state, so we're just polling
  // the current state.
  auto logging_fn = [&](std::mutex &m, std::condition_variable &cv) {
    // if the motor task is stopped, we should run the encoder update function
    // to keep the state up to date
    if (!dual_motor_timer.is_running()) {
      std::error_code ec;
      encoder1->update(ec);
      encoder2->update(ec);
    }
    static auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto seconds = std::chrono::duration<float>(now - start).count();
    auto _target1 = target1.load() * espp::RADS_TO_RPM;
    auto _target2 = target2.load() * espp::RADS_TO_RPM;
    auto rpm1 = filter1.update(motor1->get_shaft_velocity() * espp::RADS_TO_RPM);
    auto rpm2 = filter2.update(motor2->get_shaft_velocity() * espp::RADS_TO_RPM);
    auto rads1 = motor1->get_shaft_angle();
    auto rads2 = motor2->get_shaft_angle();
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
  auto logging_task = espp::Task({.name = "Logging Task",
                                  .callback = logging_fn,
                                  .stack_size_bytes = 5 * 1024,
                                  .log_level = espp::Logger::Verbosity::WARN});
  logging_task.start();

  std::this_thread::sleep_for(1s);
  logger.info("Starting target task");

  enum class IncrementDirection { DOWN = -1, HOLD = 0, UP = 1 };
  static IncrementDirection increment_direction1 = IncrementDirection::UP;
  static IncrementDirection increment_direction2 = IncrementDirection::DOWN;
  static const bool is_angle =
      motion_control_type == espp::detail::MotionControlType::ANGLE ||
      motion_control_type == espp::detail::MotionControlType::ANGLE_OPENLOOP;
  static const float max_target = is_angle ? (2.0f * M_PI) : (200.0f * espp::RPM_TO_RADS);
  static const float target_delta =
      is_angle ? (M_PI / 4.0f) : (50.0f * espp::RPM_TO_RADS * core_update_period);

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
        motor1->enable();
        motor2->enable();
        dual_motor_timer.periodic(core_update_period_us);
        target_task.start();
      } else {
        logger.info("Button released, stopping motors");
        dual_motor_timer.stop();
        target_task.stop();
        motor1->disable();
        motor2->disable();
      }
    }
    std::this_thread::sleep_for(50ms);
  }
}
