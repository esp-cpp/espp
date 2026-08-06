#define NOMINMAX // prevent Windows.h from defining min/max macros
#include "trajectory_planner.hpp"

#include "timer.hpp"
#include <chrono>
#include <cmath>
#include <thread>
#ifdef _MSC_VER
#include <windows.h>
#endif
using namespace std::chrono_literals;

static auto prog_start = std::chrono::high_resolution_clock::now();
static float elapsed() {
  return std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - prog_start)
      .count();
}

// ── helpers ──────────────────────────────────────────────────────────────────

struct Stats {
  float max_v{0}, max_w{0};
  float max_a_v_drv{0}, max_a_w_drv{0}; // driving phase peaks
  float max_a_v_stp{0}, max_a_w_stp{0}; // stopping phase peaks
  float prev_v{0}, prev_w{0};
  float prev_t{0};
  bool first{true};
  bool stopping{false};

  void set_stopping() { stopping = true; }

  void update(const espp::TrajectoryPlanner::MotionCommand &cmd, float t) {
    float v = std::abs(cmd.linear_velocity);
    float w = std::abs(cmd.angular_velocity);
    max_v = std::max(max_v, v);
    max_w = std::max(max_w, w);
    if (!first) {
      float dt = t - prev_t;
      if (dt > 0) {
        float a_v = std::abs((cmd.linear_velocity - prev_v) / dt);
        float a_w = std::abs((cmd.angular_velocity - prev_w) / dt);
        if (stopping) {
          max_a_v_stp = std::max(max_a_v_stp, a_v);
          max_a_w_stp = std::max(max_a_w_stp, a_w);
        } else {
          max_a_v_drv = std::max(max_a_v_drv, a_v);
          max_a_w_drv = std::max(max_a_w_drv, a_w);
        }
      }
    }
    prev_v = cmd.linear_velocity;
    prev_w = cmd.angular_velocity;
    prev_t = t;
    first = false;
  }

  bool check(espp::Logger &log, float drv_a_lim, float stp_a_lim, float v_lim, float tol = 1.10f) {
    bool ok = (max_v <= v_lim * 1.05f) && (max_a_v_drv <= drv_a_lim * tol) &&
              (max_a_v_stp <= stp_a_lim * tol);
    log.info("  max|v|={:.3f}/{:.1f}  |a_v_drv|={:.3f}/{:.1f}  |a_v_stp|={:.3f}/{:.1f}  -> {}",
             max_v, v_lim, max_a_v_drv, drv_a_lim, max_a_v_stp, stp_a_lim, ok ? "PASS" : "FAIL");
    return ok;
  }
};

// ── test cases ───────────────────────────────────────────────────────────────

static bool test_trapezoidal(espp::Logger &log) {
  log.info("=== Trapezoidal (accel-limited) ===");

  Stats stats;
  espp::TrajectoryPlanner planner({
      .max_linear_velocity = 1.0f,
      .max_angular_velocity = 3.14159f,
      .driving_profile = {.max_linear_acceleration = 2.0f, .max_angular_acceleration = 6.28f},
      .stopping_profile = {.max_linear_acceleration = 4.0f, .max_angular_acceleration = 8.0f},
      .output_callback =
          [&](const espp::TrajectoryPlanner::MotionCommand &cmd) { stats.update(cmd, elapsed()); },
      .planning_period = std::chrono::milliseconds(20),
      .planning_task_config = {.name = "tp_trap"},
      .log_level = espp::Logger::Verbosity::WARN,
  });

  planner.set_target(1.0f, 0.0f);
  std::this_thread::sleep_for(1s);

  stats.set_stopping();
  planner.stop();
  std::this_thread::sleep_for(500ms);

  return stats.check(log, 2.0f, 4.0f, 1.0f);
}

static bool test_scurve(espp::Logger &log) {
  log.info("=== S-curve (jerk-limited) ===");

  Stats stats;
  espp::TrajectoryPlanner planner({
      .max_linear_velocity = 1.0f,
      .max_angular_velocity = 3.14159f,
      .driving_profile = {.max_linear_acceleration = 2.0f,
                          .max_angular_acceleration = 6.28f,
                          .max_linear_jerk = 10.0f,
                          .max_angular_jerk = 25.0f},
      .stopping_profile = {.max_linear_acceleration = 5.0f, .max_angular_acceleration = 10.0f},
      .output_callback =
          [&](const espp::TrajectoryPlanner::MotionCommand &cmd) {
            stats.update(cmd, elapsed());
            log.debug("[{:.3f}] v={:+.3f}  w={:+.3f}", elapsed(), cmd.linear_velocity,
                      cmd.angular_velocity);
          },
      .planning_period = std::chrono::milliseconds(20),
      .planning_task_config = {.name = "tp_scurve"},
      .log_level = espp::Logger::Verbosity::WARN,
  });

  planner.set_target(1.0f, 0.0f);
  std::this_thread::sleep_for(1s);
  planner.set_target(0.5f, -0.4f);
  std::this_thread::sleep_for(1s);
  stats.set_stopping();
  planner.stop();
  std::this_thread::sleep_for(500ms);

  return stats.check(log, 2.0f, 5.0f, 1.0f);
}

static bool test_direction_reversal(espp::Logger &log) {
  log.info("=== Direction reversal ===");

  float peak_v = 0;
  espp::TrajectoryPlanner planner({
      .max_linear_velocity = 1.0f,
      .max_angular_velocity = 3.14159f,
      .driving_profile = {.max_linear_acceleration = 2.0f,
                          .max_angular_acceleration = 6.28f,
                          .max_linear_jerk = 10.0f,
                          .max_angular_jerk = 25.0f},
      .stopping_profile = {.max_linear_acceleration = 5.0f, .max_angular_acceleration = 10.0f},
      .output_callback =
          [&](const espp::TrajectoryPlanner::MotionCommand &cmd) {
            peak_v = std::max(peak_v, std::abs(cmd.linear_velocity));
          },
      .planning_period = std::chrono::milliseconds(20),
      .planning_task_config = {.name = "tp_rev"},
      .log_level = espp::Logger::Verbosity::WARN,
  });

  planner.set_target(1.0f, 0.0f);
  std::this_thread::sleep_for(600ms);
  planner.set_target(-1.0f, 0.0f);
  std::this_thread::sleep_for(1200ms);
  planner.stop();
  std::this_thread::sleep_for(500ms);

  bool ok = peak_v <= 1.0f * 1.05f;
  log.info("  peak|v|={:.3f}/1.0  -> {}", peak_v, ok ? "PASS" : "FAIL");
  return ok;
}

static bool test_reset(espp::Logger &log) {
  log.info("=== Emergency reset ===");

  espp::TrajectoryPlanner planner({
      .max_linear_velocity = 1.0f,
      .max_angular_velocity = 3.14159f,
      .driving_profile = {.max_linear_acceleration = 2.0f, .max_angular_acceleration = 6.28f},
      .stopping_profile = {.max_linear_acceleration = 5.0f, .max_angular_acceleration = 10.0f},
      .planning_period = std::chrono::milliseconds(20),
      .planning_task_config = {.name = "tp_reset"},
      .log_level = espp::Logger::Verbosity::WARN,
  });

  planner.set_target(1.0f, 0.5f);
  std::this_thread::sleep_for(600ms);
  planner.reset();

  auto cmd = planner.output();
  bool ok = (cmd.linear_velocity == 0.0f) && (cmd.angular_velocity == 0.0f);
  log.info("  post-reset v={:.3f} w={:.3f}  -> {}", cmd.linear_velocity, cmd.angular_velocity,
           ok ? "PASS" : "FAIL");
  return ok;
}

// ── main ─────────────────────────────────────────────────────────────────────

int main() {
  espp::Logger log({.tag = "TrajectoryPlanner Test", .level = espp::Logger::Verbosity::INFO});
  log.info("Starting trajectory planner tests");

#ifdef _MSC_VER
  log.info("On Windows, setting timeBeginPeriod(1)");
  if (timeBeginPeriod(1) == TIMERR_NOERROR) {
    log.info("Success");
  } else {
    log.error("Failed to set timeBeginPeriod(1)");
  }
#endif

  int passed = 0, total = 0;
  auto run = [&](bool result) {
    ++total;
    if (result)
      ++passed;
  };

  run(test_trapezoidal(log));
  run(test_scurve(log));
  run(test_direction_reversal(log));
  run(test_reset(log));

  log.info("Results: {}/{} passed", passed, total);
  return (passed == total) ? 0 : 1;
}
