#include <atomic>
#include <chrono>

#include "trajectory_planner.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "TrajectoryPlanner Example", .level = espp::Logger::Verbosity::INFO});

  {
    logger.info("=== Example 1: Trapezoidal profile with built-in task ===");
    //! [trajectory_planner example]
    std::atomic<int> tick{0};

    espp::TrajectoryPlanner planner({
        .max_linear_velocity = 1.0f,       // m/s
        .max_angular_velocity = 3.14159f,  // rad/s
        .max_linear_acceleration = 2.0f,   // m/s²  — driving ramp
        .max_angular_acceleration = 6.28f, // rad/s²
        .max_linear_deceleration = 6.0f,   // m/s²  — 3× faster braking stop
        .max_angular_deceleration = 12.0f, // rad/s²
        .enforce_motion_envelope = true,
        .max_centripetal_acceleration = 0.5f, // m/s²
        .output_callback =
            [&logger, &tick](const espp::TrajectoryPlanner::MotionCommand &cmd) {
              logger.info("[{:3d}] {}", tick.load(), cmd);
              tick++;
            },
        .update_period = 20ms, // 50 Hz
        .auto_start = true,
        .task_config =
            {.name = "TrajPlanner1", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });

    // Full forward (normalized: 1.0 = max_linear_velocity)
    logger.info("Commanding full forward");
    planner.set_target(1.0f, 0.0f);
    std::this_thread::sleep_for(1s);

    // Gentle right curve at half speed
    logger.info("Commanding half-speed right curve");
    planner.set_target(0.5f, -0.4f);
    std::this_thread::sleep_for(1s);

    // Controlled stop — uses braking deceleration limits
    logger.info("Stopping");
    planner.stop();
    std::this_thread::sleep_for(500ms);

    planner.stop_task();
    //! [trajectory_planner example]
  }

  {
    logger.info("=== Example 2: S-curve (jerk-limited) with combined motion ===");
    //! [trajectory_planner jerk example]
    std::atomic<int> tick{0};

    espp::TrajectoryPlanner planner({
        .max_linear_velocity = 4.47f,            // 10 mph
        .max_angular_velocity = 3.14159f / 2.0f, // 90 deg/s
        .max_linear_acceleration = 2.0f,         // m/s²
        .max_angular_acceleration = 6.28f,       // rad/s²
        .max_linear_deceleration = 4.0f,         // m/s²
        .max_angular_deceleration = 8.0f,        // rad/s²
        .max_linear_jerk = 10.0f,                // m/s³ — enables S-curve mode
        .max_angular_jerk = 30.0f,               // rad/s³
        .enforce_motion_envelope = true,
        .max_centripetal_acceleration = 0.3f,
        .output_callback =
            [&logger, &tick](const espp::TrajectoryPlanner::MotionCommand &cmd) {
              logger.info("[{:3d}] {}", tick.load(), cmd);
              tick++;
            },
        .update_period = 20ms,
        .auto_start = true,
        .task_config =
            {.name = "TrajPlanner2", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });

    // Forward + left turn (0.8 = 80% max linear, 0.5 = 50% max angular)
    logger.info("Commanding forward + left turn");
    planner.set_target(0.8f, 0.5f);
    std::this_thread::sleep_for(1s);

    // Reverse direction
    logger.info("Reversing direction");
    planner.set_target(-0.5f, -0.5f);
    std::this_thread::sleep_for(1s);

    // Controlled stop
    logger.info("Stopping");
    planner.stop();
    std::this_thread::sleep_for(500ms);

    planner.stop_task();
    //! [trajectory_planner jerk example]
  }

  logger.info("Example complete");
}
