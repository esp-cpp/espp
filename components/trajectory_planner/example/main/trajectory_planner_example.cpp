#include <atomic>
#include <chrono>

#include "trajectory_planner.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "TrajectoryPlanner Example", .level = espp::Logger::Verbosity::INFO});

  // ---------------------------------------------------------------------------
  // Quick-start: all public API in one place
  // ---------------------------------------------------------------------------
  {
    logger.info("=== Quick-start: public API overview ===");
    //! [trajectory_planner quickstart]

    // 1. Construct with a Config — task starts automatically.
    espp::TrajectoryPlanner planner({
        .max_linear_velocity = 1.0f,      // m/s
        .max_angular_velocity = 3.14159f, // rad/s
        .driving_profile = {.max_linear_acceleration = 2.0f,
                            .max_angular_acceleration = 6.28f,
                            .max_linear_jerk = 8.0f,
                            .max_angular_jerk = 20.0f},
        // Trapezoidal stop (no jerk) — fast, clean, no overshoot
        .stopping_profile = {.max_linear_acceleration = 5.0f, .max_angular_acceleration = 10.0f},
        .enforce_motion_envelope = true,      // keep (v/vmax)²+(ω/ωmax)²≤1
        .max_centripetal_acceleration = 0.4f, // m/s²
        .output_callback =
            [&logger](const espp::TrajectoryPlanner::MotionCommand &cmd) {
              logger.debug("callback: {}", cmd);
            },
        .update_period = 20ms,
        .task_config = {.name = "QuickStart", .stack_size_bytes = 8192},
    });

    // 2. is_running() — confirm the task started.
    logger.info("Task running: {}", planner.is_running());

    // 3. get_config() — inspect active configuration.
    auto cfg = planner.get_config();
    logger.info("Config: {}", cfg);

    // 4. set_target(linear, angular) — normalized [-1, +1] joystick inputs.
    //    +1.0 linear = max_linear_velocity forward.
    planner.set_target(1.0f, 0.0f);
    std::this_thread::sleep_for(600ms);

    // 5. output() — poll the latest smoothed command at any time.
    auto cmd = planner.output();
    logger.info("Polled output: {}", cmd);

    // 6. set_target with combined motion — forward + right turn.
    planner.set_target(0.6f, -0.5f);
    std::this_thread::sleep_for(600ms);

    // 7. set_config() — change parameters at runtime; resets state by default.
    espp::TrajectoryPlanner::Config new_cfg = planner.get_config();
    new_cfg.max_linear_velocity = 0.5f; // half speed cap
    planner.set_config(new_cfg, /*reset_state=*/false);
    logger.info("Updated max_linear_velocity to 0.5 m/s");
    planner.set_target(1.0f, 0.0f); // still clamped to new 0.5 m/s
    std::this_thread::sleep_for(600ms);

    // 8. stop() — ramp down to zero respecting deceleration limits.
    logger.info("Commanding stop (ramp-down)");
    planner.stop();
    std::this_thread::sleep_for(400ms);

    // 9. reset() — zero state immediately (e.g. after e-stop).
    logger.info("Emergency reset");
    planner.reset();
    logger.info("Output after reset: {}", planner.output());

    // 10. Destructor stops the task automatically when planner leaves scope.
    //! [trajectory_planner quickstart]
  }

  {
    //! [trajectory_planner example]
    std::atomic<int> tick{0};

    espp::TrajectoryPlanner planner({
        .max_linear_velocity = 1.0f,      // m/s
        .max_angular_velocity = 3.14159f, // rad/s
        // S-curve driving: smooth ramp with jerk limiting
        .driving_profile = {.max_linear_acceleration = 2.0f,
                            .max_angular_acceleration = 6.28f,
                            .max_linear_jerk = 10.0f,
                            .max_angular_jerk = 25.0f},
        // Trapezoidal stop: no jerk = immediate deceleration, no overshoot
        .stopping_profile = {.max_linear_acceleration = 6.0f, .max_angular_acceleration = 12.0f},
        .enforce_motion_envelope = true,
        .max_centripetal_acceleration = 0.5f, // m/s²
        .output_callback =
            [&logger, &tick](const espp::TrajectoryPlanner::MotionCommand &cmd) {
              logger.info("[{:3d}] {}", tick.load(), cmd);
              tick++;
            },
        .update_period = 20ms, // 50 Hz
        .task_config =
            {.name = "TrajPlanner1", .stack_size_bytes = 8192, .priority = 5, .core_id = -1},
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
    // task stops automatically when planner goes out of scope
    //! [trajectory_planner example]
  }

  {
    logger.info("=== Example 2: S-curve (jerk-limited) with combined motion ===");
    //! [trajectory_planner jerk example]
    std::atomic<int> tick{0};

    espp::TrajectoryPlanner planner({
        .max_linear_velocity = 4.47f,            // 10 mph
        .max_angular_velocity = 3.14159f / 2.0f, // 90 deg/s
        // S-curve driving: smooth acceleration with jerk limits
        .driving_profile = {.max_linear_acceleration = 2.0f,
                            .max_angular_acceleration = 6.28f,
                            .max_linear_jerk = 10.0f,
                            .max_angular_jerk = 30.0f},
        // Trapezoidal stop: no jerk limit = clean stop, no overshoot
        .stopping_profile = {.max_linear_acceleration = 4.0f, .max_angular_acceleration = 8.0f},
        .enforce_motion_envelope = true,
        .max_centripetal_acceleration = 0.3f,
        .output_callback =
            [&logger, &tick](const espp::TrajectoryPlanner::MotionCommand &cmd) {
              logger.info("[{:3d}] {}", tick.load(), cmd);
              tick++;
            },
        .update_period = 20ms,
        .task_config =
            {.name = "TrajPlanner2", .stack_size_bytes = 8192, .priority = 5, .core_id = -1},
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
    // task stops automatically when planner goes out of scope
    //! [trajectory_planner jerk example]
  }

  logger.info("Example complete");
}
