#include <chrono>

#include "task.hpp"
#include "trajectory_planner.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    fmt::print("=== Acceleration-limited (trapezoidal) example ===\n");
    //! [trajectory_planner example]
    espp::TrajectoryPlanner planner({
        .max_linear_velocity = 1.0f,       // m/s
        .max_angular_velocity = 3.14159f,  // rad/s
        .max_linear_acceleration = 2.0f,   // m/s²
        .max_angular_acceleration = 6.28f, // rad/s²
        .max_linear_jerk = 0.0f,           // 0 = acceleration-limited mode
        .max_angular_jerk = 0.0f,
        .enforce_motion_envelope = true,
        .log_level = espp::Logger::Verbosity::WARN,
    });

    constexpr float dt = 0.02f; // 50 Hz control loop
    constexpr int steps = 100;

    // Ramp up to full forward speed
    planner.set_target(1.0f, 0.0f);
    for (int i = 0; i < steps; i++) {
      planner.update(dt);
      auto cmd = planner.output();
      fmt::print("[{:3d}] {}\n", i, cmd);
      // At halfway, command a controlled stop
      if (i == steps / 2) {
        planner.stop();
      }
    }
    //! [trajectory_planner example]
  }

  {
    fmt::print("\n=== Jerk-limited (S-curve) example ===\n");
    //! [trajectory_planner jerk example]
    espp::TrajectoryPlanner planner({
        .max_linear_velocity = 1.0f,
        .max_angular_velocity = 3.14159f,
        .max_linear_acceleration = 2.0f,
        .max_angular_acceleration = 6.28f,
        .max_linear_jerk = 10.0f,  // m/s³ — enables S-curve mode
        .max_angular_jerk = 30.0f, // rad/s³
        .enforce_motion_envelope = false,
        .log_level = espp::Logger::Verbosity::WARN,
    });

    constexpr float dt = 0.02f;
    constexpr int steps = 150;

    // Command a combined linear + angular motion
    planner.set_target(0.8f, 1.0f);
    for (int i = 0; i < steps; i++) {
      planner.update(dt);
      auto cmd = planner.output();
      fmt::print("[{:3d}] {}\n", i, cmd);
      if (i == steps / 2) {
        // Reverse direction
        planner.set_target(-0.5f, -0.5f);
      }
    }
    //! [trajectory_planner jerk example]
  }
}
