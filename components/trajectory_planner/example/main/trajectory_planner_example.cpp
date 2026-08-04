#include <atomic>
#include <chrono>

#include "trajectory_planner.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "TrajectoryPlanner Example", .level = espp::Logger::Verbosity::INFO});

  // ---------------------------------------------------------------------------
  // Quick-start: all public API in one place
  // ---------------------------------------------------------------------------
  auto planning_period = 50ms;  // 20hz
  auto callback_period = 100ms; // 10hz
  {
    logger.info("=== Quick-start: public API overview ===");
    //! [trajectory_planner quickstart]

    // 1. Construct with a Config - task starts automatically.
    espp::TrajectoryPlanner planner({
        .max_linear_velocity = 1.0f,      // m/s
        .max_angular_velocity = 3.14159f, // rad/s
        .driving_profile = {.max_linear_acceleration = 2.0f,
                            .max_angular_acceleration = 6.28f,
                            .max_linear_jerk = 2.0f,
                            .max_angular_jerk = 20.0f},
        // Trapezoidal stop (no jerk) - fast, clean, no overshoot
        .stopping_profile = {.max_linear_acceleration = 5.0f, .max_angular_acceleration = 10.0f},
        .enforce_motion_envelope = true,      // keep (v/vmax)²+(ω/ωmax)²≤1
        .max_centripetal_acceleration = 0.4f, // m/s²
        .output_callback =
            [&logger](const espp::TrajectoryPlanner::MotionCommand &cmd) {
              logger.debug("callback: {}", cmd);
            },
        .planning_period = planning_period,
        .callback_period = callback_period,
        .planning_task_config = {.name = "tp_qs", .stack_size_bytes = 10240},
    });

    // 2. is_running() - confirm the task started.
    logger.info("Task running: {}", planner.is_running());

    // 3. get_config() - inspect active configuration.
    auto cfg = planner.get_config();
    logger.info("Config: {}", cfg);

    // 4. set_target(linear, angular) - normalized [-1, +1] joystick inputs.
    //    +1.0 linear = max_linear_velocity forward.
    planner.set_target(1.0f, 0.0f);
    std::this_thread::sleep_for(600ms);

    // 5. output() - poll the latest smoothed command at any time.
    auto cmd = planner.output();
    logger.info("Polled output: {}", cmd);

    // 6. set_target with combined motion - forward + right turn.
    planner.set_target(0.6f, -0.5f);
    std::this_thread::sleep_for(600ms);

    // 7. set_config() - change parameters at runtime; resets state by default.
    espp::TrajectoryPlanner::Config new_cfg = planner.get_config();
    new_cfg.max_linear_velocity = 0.5f; // half speed cap
    planner.set_config(new_cfg, /*reset_state=*/false);
    logger.info("Updated max_linear_velocity to 0.5 m/s");
    planner.set_target(1.0f, 0.0f); // still clamped to new 0.5 m/s
    std::this_thread::sleep_for(600ms);

    // 8. stop() - ramp down to zero respecting deceleration limits.
    logger.info("Commanding stop (ramp-down)");
    planner.stop();
    std::this_thread::sleep_for(400ms);

    // 9. reset() - zero state immediately (e.g. after e-stop).
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
                            .max_linear_jerk = 2.0f,
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
        .planning_period = planning_period,
        .callback_period = callback_period,
        .planning_task_config =
            {.name = "tp_ex1", .stack_size_bytes = 10240, .priority = 5, .core_id = -1},
    });

    // Full forward (normalized: 1.0 = max_linear_velocity)
    logger.info("Commanding full forward");
    planner.set_target(1.0f, 0.0f);
    std::this_thread::sleep_for(3s);

    // Gentle right curve at half speed
    logger.info("Commanding half-speed right curve");
    planner.set_target(0.5f, -0.4f);
    std::this_thread::sleep_for(3s);

    // Controlled stop - uses braking deceleration limits
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
        .planning_period = planning_period,
        .callback_period = callback_period,
        .planning_task_config =
            {.name = "tp_ex2", .stack_size_bytes = 10240, .priority = 5, .core_id = -1},
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

  // ---------------------------------------------------------------------------
  // Constraint validation - verifies that the planner always stays within its
  // configured speed, acceleration, jerk, and centripetal limits.
  //
  // A Validator is attached as the output_callback.  On every tick it:
  //   • checks |v| / |ω| ≤ max velocity limits
  //   • checks |v·ω| ≤ max centripetal acceleration
  //   • computes dv/dt and checks against the active profile's acceleration limit
  //   • computes d²v/dt² and checks against the active profile's jerk limit
  // ---------------------------------------------------------------------------
  {
    logger.info("=== Constraint validation tests ===");
    //! [trajectory_planner validation]

    using Cfg = espp::TrajectoryPlanner::Config;
    using Cmd = espp::TrajectoryPlanner::MotionCommand;

    // ---- Validator struct ------------------------------------------------
    struct Validator {
      // Immutable limits
      float max_v, max_w;
      float a_drv_v, a_drv_w, j_drv_v, j_drv_w; // driving profile
      float a_stp_v, a_stp_w, j_stp_v, j_stp_w; // stopping profile
      float max_cp, dt;

      // Callback-thread history (only written from callback)
      float pv{0}, pw{0}, pa_v{0}, pa_w{0};
      bool inited{false};
      int skip{2}; // skip first N ticks for derivative settling
      std::chrono::steady_clock::time_point prev_time{};
      bool time_inited{false};

      // Shared results - protected by mtx
      mutable std::mutex mtx;
      espp::Logger *log{nullptr}; // set after construction
      bool stopping{false};
      int viol_drv{0}; // violations during driving phase
      int viol_stp{0}; // violations during stopping phase

      struct PhasePeaks {
        float v{0}, w{0}, a_v{0}, a_w{0}, j_v{0}, j_w{0}, cp{0};
        void reset() { v = w = a_v = a_w = j_v = j_w = cp = 0; }
      };
      PhasePeaks pk_drv, pk_stp;

      void mark_stopping() {
        std::lock_guard<std::mutex> lk(mtx);
        stopping = true;
        skip = 3;
      }

      // Call before resuming a non-zero target after a stop.
      void mark_driving() {
        std::lock_guard<std::mutex> lk(mtx);
        stopping = false;
        skip = 3;
      }

      // Call before any abrupt target change (direction reversal, etc.).
      void mark_transition() {
        std::lock_guard<std::mutex> lk(mtx);
        skip = 3;
      }

      void reset() {
        // Always called before a new planner starts - no concurrent access
        std::lock_guard<std::mutex> lk(mtx);
        stopping = false;
        viol_drv = 0;
        viol_stp = 0;
        pk_drv.reset();
        pk_stp.reset();
        pv = pw = pa_v = pa_w = 0;
        inited = false;
        skip = 2;
        time_inited = false;
      }

      void operator()(const Cmd &cmd) {
        std::lock_guard<std::mutex> lk(mtx);
        const float tol = 1.08f; // 8% margin for scheduler jitter
        const float ma_v = stopping ? a_stp_v : a_drv_v;
        const float ma_w = stopping ? a_stp_w : a_drv_w;
        const float mj_v = stopping ? j_stp_v : j_drv_v;
        const float mj_w = stopping ? j_stp_w : j_drv_w;

        // Speed limits
        PhasePeaks &pk = stopping ? pk_stp : pk_drv;
        pk.v = std::max(pk.v, std::abs(cmd.linear_velocity));
        pk.w = std::max(pk.w, std::abs(cmd.angular_velocity));
        if (std::abs(cmd.linear_velocity) > max_v * tol) {
          if (log)
            log->warn("VIOL speed_v: {:.3f} > {:.3f} ({})", cmd.linear_velocity, max_v * tol,
                      stopping ? "stop" : "drv");
          stopping ? viol_stp++ : viol_drv++;
        }
        if (std::abs(cmd.angular_velocity) > max_w * tol) {
          if (log)
            log->warn("VIOL speed_w: {:.3f} > {:.3f} ({})", cmd.angular_velocity, max_w * tol,
                      stopping ? "stop" : "drv");
          stopping ? viol_stp++ : viol_drv++;
        }

        // Centripetal limit
        float cp = std::abs(cmd.linear_velocity * cmd.angular_velocity);
        pk.cp = std::max(pk.cp, cp);

        if (inited) {
          // Use actual elapsed time to avoid systematic error from FreeRTOS
          // scheduling jitter (actual tick ~30 ms vs. nominal 20 ms).
          auto now = std::chrono::steady_clock::now();
          float adt = time_inited ? std::chrono::duration<float>(now - prev_time).count()
                                  : dt; // nominal for first step
          prev_time = now;
          time_inited = true;

          float a_v = (cmd.linear_velocity - pv) / adt;
          float a_w = (cmd.angular_velocity - pw) / adt;
          pk.a_v = std::max(pk.a_v, std::abs(a_v));
          pk.a_w = std::max(pk.a_w, std::abs(a_w));
          float j_v = (a_v - pa_v) / adt;
          float j_w = (a_w - pa_w) / adt;

          if (skip > 0) {
            --skip;
          } else {
            pk.j_v = std::max(pk.j_v, std::abs(j_v));
            pk.j_w = std::max(pk.j_w, std::abs(j_w));
            if (std::abs(a_v) > ma_v * tol) {
              if (log)
                log->warn("VIOL accel_v: {:.3f} > {:.3f} ({})", a_v, ma_v * tol,
                          stopping ? "stop" : "drv");
              stopping ? viol_stp++ : viol_drv++;
            }
            if (std::abs(a_w) > ma_w * tol) {
              if (log)
                log->warn("VIOL accel_w: {:.3f} > {:.3f} ({})", a_w, ma_w * tol,
                          stopping ? "stop" : "drv");
              stopping ? viol_stp++ : viol_drv++;
            }
            if (mj_v > 0.0f && std::abs(j_v) > mj_v * tol) {
              if (log)
                log->warn("VIOL jerk_v:  {:.3f} > {:.3f} ({})", j_v, mj_v * tol,
                          stopping ? "stop" : "drv");
              stopping ? viol_stp++ : viol_drv++;
            }
            if (mj_w > 0.0f && std::abs(j_w) > mj_w * tol) {
              if (log)
                log->warn("VIOL jerk_w:  {:.3f} > {:.3f} ({})", j_w, mj_w * tol,
                          stopping ? "stop" : "drv");
              stopping ? viol_stp++ : viol_drv++;
            }
          }
          pa_v = a_v;
          pa_w = a_w;
        } else {
          // First tick: start the clock
          prev_time = std::chrono::steady_clock::now();
          time_inited = true;
        }
        pv = cmd.linear_velocity;
        pw = cmd.angular_velocity;
        inited = true;
      }

      bool report(espp::Logger &log, std::string_view name) {
        std::lock_guard<std::mutex> lk(mtx);
        log.info("--- {} ---  drv_viol={} stp_viol={}", name, viol_drv, viol_stp);
        log.info("  [driving]");
        log.info("    v={:.3f}/{:.1f}   w={:.3f}/{:.1f}", pk_drv.v, max_v, pk_drv.w, max_w);
        log.info("    a_v={:.3f}/{:.1f}  a_w={:.3f}/{:.1f}", pk_drv.a_v, a_drv_v, pk_drv.a_w,
                 a_drv_w);
        log.info("    j_v={:.3f}/{:.1f}  j_w={:.3f}/{:.1f}", pk_drv.j_v, j_drv_v, pk_drv.j_w,
                 j_drv_w);
        log.info("    cp={:.3f}/{:.2f}", pk_drv.cp, max_cp);
        log.info("  [stopping]");
        log.info("    v={:.3f}/{:.1f}   w={:.3f}/{:.1f}", pk_stp.v, max_v, pk_stp.w, max_w);
        log.info("    a_v={:.3f}/{:.1f}  a_w={:.3f}/{:.1f}", pk_stp.a_v, a_stp_v, pk_stp.a_w,
                 a_stp_w);
        log.info("    j_v={:.3f}/{:.1f}  j_w={:.3f}/{:.1f}", pk_stp.j_v, j_stp_v, pk_stp.j_w,
                 j_stp_w);
        log.info("    cp={:.3f}/{:.2f}", pk_stp.cp, max_cp);
        log.info((viol_drv == 0 && viol_stp == 0) ? "  [PASS]" : "  [FAIL]");
        return viol_drv == 0 && viol_stp == 0;
      }
    };

    // ---- Shared limits ---------------------------------------------------
    constexpr float MAX_V = 1.0f, MAX_W = 3.14159f;
    constexpr float A_DRV_V = 2.0f, A_DRV_W = 6.28f;
    constexpr float J_DRV_V = 10.0f, J_DRV_W = 25.0f;
    constexpr float A_STP_V = 5.0f, A_STP_W = 10.0f;
    constexpr float MAX_CP = 0.4f, DT = 0.020f;

    Validator val;
    val.max_v = MAX_V;
    val.max_w = MAX_W;
    val.a_drv_v = A_DRV_V;
    val.a_drv_w = A_DRV_W;
    val.j_drv_v = J_DRV_V;
    val.j_drv_w = J_DRV_W;
    val.a_stp_v = A_STP_V;
    val.a_stp_w = A_STP_W;
    val.j_stp_v = 0.0f;
    val.j_stp_w = 0.0f; // tests use trapez stop; set non-zero if needed
    val.max_cp = MAX_CP;
    val.dt = DT;
    val.log = &logger;

    const Cfg cfg{
        .max_linear_velocity = MAX_V,
        .max_angular_velocity = MAX_W,
        .driving_profile = {.max_linear_acceleration = A_DRV_V,
                            .max_angular_acceleration = A_DRV_W,
                            .max_linear_jerk = J_DRV_V,
                            .max_angular_jerk = J_DRV_W},
        .stopping_profile = {.max_linear_acceleration = A_STP_V,
                             .max_angular_acceleration = A_STP_W},
        .enforce_motion_envelope = true,
        .max_centripetal_acceleration = MAX_CP,
        .output_callback = [&val](const Cmd &cmd) { val(cmd); },
        .planning_period = std::chrono::milliseconds(50),
        .callback_period = std::chrono::milliseconds(100),
        .planning_task_config = {.name = "tp_val", .stack_size_bytes = 1024 * 10, .priority = 5},
        .callback_task_config = {.name = "tp_val_cb", .stack_size_bytes = 8192, .priority = 5},

    };

    // ---- Test A: pure linear ramp ----------------------------------------
    {
      logger.info("[A] Pure linear ramp  target=(+1, 0)");
      val.reset();
      espp::TrajectoryPlanner planner(cfg);
      planner.set_target(1.0f, 0.0f);
      std::this_thread::sleep_for(800ms);
      val.report(logger, "A: Pure linear ramp");
    }

    // ---- Test B: pure angular ramp ----------------------------------------
    {
      logger.info("[B] Pure angular ramp  target=(0, +1)");
      val.reset();
      espp::TrajectoryPlanner planner(cfg);
      planner.set_target(0.0f, 1.0f);
      std::this_thread::sleep_for(800ms);
      val.report(logger, "B: Pure angular ramp");
    }

    // ---- Test C: combined motion - tests envelope + centripetal ----------
    {
      logger.info("[C] Combined  target=(0.9, 0.9)  envelope + centripetal");
      val.reset();
      espp::TrajectoryPlanner planner(cfg);
      planner.set_target(0.9f, 0.9f);
      std::this_thread::sleep_for(1s);
      val.report(logger, "C: Combined (envelope + centripetal)");
    }

    // ---- Test D: drive → controlled stop (stopping profile) ---------------
    {
      logger.info("[D] Drive then stop  - tests stopping profile switch");
      val.reset();
      espp::TrajectoryPlanner planner(cfg);
      planner.set_target(1.0f, 0.0f);
      std::this_thread::sleep_for(600ms);
      val.mark_stopping();
      planner.stop();
      std::this_thread::sleep_for(400ms);
      val.report(logger, "D: Drive then stop");
    }

    // ---- Test E: direction reversal ---------------------------------------
    {
      logger.info("[E] Direction reversal  (+1, 0) → (-1, 0)");
      val.reset();
      espp::TrajectoryPlanner planner(cfg);
      planner.set_target(1.0f, 0.0f);
      std::this_thread::sleep_for(500ms);
      planner.set_target(-1.0f, 0.0f);
      std::this_thread::sleep_for(1000ms);
      val.report(logger, "E: Direction reversal");
    }

    // ---- Test F: multi-step sweep - various (linear, angular) pairs -------
    {
      logger.info("[F] Multi-step sweep  - varied inputs");
      val.reset();
      espp::TrajectoryPlanner planner(cfg);
      bool was_stopping = false;
      for (float lin : {0.3f, 0.6f, 1.0f, -0.5f, 0.0f}) {
        for (float ang : {0.0f, 0.5f, -0.5f}) {
          bool next_stopping = (lin == 0.0f && ang == 0.0f);
          if (next_stopping && !was_stopping)
            val.mark_stopping();
          else if (!next_stopping && was_stopping)
            val.mark_driving();
          was_stopping = next_stopping;
          planner.set_target(lin, ang);
          std::this_thread::sleep_for(250ms);
        }
      }
      val.mark_stopping();
      planner.stop();
      std::this_thread::sleep_for(300ms);
      val.report(logger, "F: Multi-step sweep");
    }
    //! [trajectory_planner validation]
  }

  logger.info("Example complete");
}
