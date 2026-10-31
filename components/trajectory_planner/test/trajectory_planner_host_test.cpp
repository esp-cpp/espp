// Host-buildable test for the TrajectoryPlanner's per-axis maths
// (include/detail/trajectory_planner_math.hpp -- standard library only).
// Steps one axis through the planner's profile hand-offs exactly as
// TrajectoryPlanner::update() does. Build & run:
//   c++ -std=c++23 -Wall -Wextra -Werror -I components/trajectory_planner/include \
//       components/trajectory_planner/test/trajectory_planner_host_test.cpp -o test && ./test

#include <cmath>
#include <cstdio>

#include "detail/trajectory_planner_math.hpp"

namespace tp = espp::detail::trajectory_planner;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

static bool near(float a, float b, float tol = 1e-4f) { return std::abs(a - b) <= tol; }

// One axis with the planner's profile selection: a driving profile while the
// target is non-zero, a stopping profile while it is zero (as update() does).
struct Axis {
  float v_max = 1.0f, a_drive = 1.0f, j_drive = 0.0f, a_stop = 1.0f, j_stop = 0.0f, dt = 0.02f;
  tp::AxisState s{};
  float max_jerk_seen = 0.0f; // |delta a| / dt over every step
  void step(float target) {
    const bool stopping = target == 0.0f;
    const float a_prev = s.a;
    tp::step_axis(s, target, -v_max, v_max, stopping ? a_stop : a_drive,
                  stopping ? j_stop : j_drive, dt);
    max_jerk_seen = std::max(max_jerk_seen, std::abs(s.a - a_prev) / dt);
  }
  void run(float target, int steps) {
    for (int i = 0; i < steps; i++)
      step(target);
  }
};

int main() {
  // ---- the scenario of issue #850: S-curve driving, trapezoidal stopping ----
  // driving a 1.0 m/s^2 / 5 m/s^3, stopping 1.6 m/s^2 trapezoidal, 50 Hz
  {
    Axis ax{.v_max = 1.8f,
            .a_drive = 1.0f,
            .j_drive = 5.0f,
            .a_stop = 1.6f,
            .j_stop = 0.0f,
            .dt = 0.02f};
    ax.run(1.8f, 25); // drive 0.5 s: a ramps 0.1 .. 1.0 over 10 steps, then holds 1.0
    CHECK(near(ax.s.v, 0.41f) && near(ax.s.a, 1.0f));
    ax.run(0.0f, 25); // stop 0.5 s (needs 13 steps at 1.6 m/s^2)
    CHECK(near(ax.s.v, 0.0f));
    CHECK(near(ax.s.a, 0.0f)); // the stale 1.0 of the report is gone
    // reverse: the first step must already go backwards, at the jerk limit (-0.1 m/s^2)
    ax.step(-1.8f);
    CHECK(near(ax.s.a, -0.1f) && ax.s.v < 0.0f);
    float v_peak = 0.0f;
    for (int i = 0; i < 24; i++) {
      ax.step(-1.8f);
      v_peak = std::max(v_peak, ax.s.v);
    }
    CHECK(v_peak <= 0.0f); // never moved forward first (the report saw +0.090 m/s)
    // forward again from standstill: the first step is jerk-limited, not a jump to 1.0
    ax.run(0.0f, 50);
    CHECK(near(ax.s.v, 0.0f) && near(ax.s.a, 0.0f));
    ax.step(1.8f);
    CHECK(near(ax.s.a, 0.1f));
  }

  // ---- the jerk limit holds across the whole stop-and-go cycle in driving mode ----
  // (the trapezoidal stop itself is allowed its own acceleration step)
  {
    Axis ax{.v_max = 1.8f,
            .a_drive = 1.0f,
            .j_drive = 5.0f,
            .a_stop = 1.6f,
            .j_stop = 0.0f,
            .dt = 0.02f};
    ax.run(1.8f, 25);
    ax.run(0.0f, 25);
    ax.max_jerk_seen = 0.0f;
    ax.run(-1.8f, 100);
    ax.run(1.8f, 100);
    CHECK(ax.max_jerk_seen <= 5.0f + 1e-3f);
  }

  // ---- a stop interrupted mid-way: the S-curve resumes from the real deceleration ----
  // and winds it back into its own limit at the jerk limit (no 1.6 -> 1.0 jump)
  {
    Axis ax{.v_max = 1.8f,
            .a_drive = 1.0f,
            .j_drive = 5.0f,
            .a_stop = 1.6f,
            .j_stop = 0.0f,
            .dt = 0.02f};
    ax.run(1.8f, 40); // v = 0.71
    ax.run(0.0f, 5);  // braking at -1.6 m/s^2 for 0.1 s
    CHECK(near(ax.s.a, -1.6f));
    ax.max_jerk_seen = 0.0f;
    ax.step(1.8f); // push the stick again: a moves from -1.6 by at most 0.1
    CHECK(near(ax.s.a, -1.5f));
    ax.run(1.8f, 60);
    CHECK(ax.max_jerk_seen <= 5.0f + 1e-3f);
    CHECK(ax.s.a <= 1.0f + 1e-6f);
  }

  // ---- both profiles trapezoidal: unchanged behaviour, acceleration recorded ----
  {
    Axis ax{.v_max = 1.8f,
            .a_drive = 1.0f,
            .j_drive = 0.0f,
            .a_stop = 1.6f,
            .j_stop = 0.0f,
            .dt = 0.02f};
    ax.step(1.8f);
    CHECK(near(ax.s.v, 0.02f) && near(ax.s.a, 1.0f));
    ax.run(1.8f, 200);
    CHECK(near(ax.s.v, 1.8f) && near(ax.s.a, 0.0f)); // at the velocity limit: no acceleration
    ax.run(0.0f, 100);
    CHECK(near(ax.s.v, 0.0f) && near(ax.s.a, 0.0f));
  }

  // ---- a profile jerk-limited on one axis only: the other axis is trapezoidal ----
  // (it used to be fed through the S-curve logic with a huge "jerk" and never moved)
  {
    tp::AxisState s{};
    tp::step_axis(s, 3.0f, -3.0f, 3.0f, 6.0f, 0.0f, 0.02f);
    CHECK(near(s.v, 0.12f) && near(s.a, 6.0f));
  }

  // ---- asymmetric velocity bounds (issue #852): full reverse settles at the reverse limit ----
  {
    tp::AxisState s{};
    for (int i = 0; i < 100; i++)
      tp::step_axis(s, -1.8f, -1.0f, 1.8f, 1.6f, 0.0f, 0.02f); // target beyond the bound
    CHECK(near(s.v, -1.0f) && near(s.a, 0.0f));
    for (int i = 0; i < 100; i++)
      tp::step_axis(s, 1.8f, -1.0f, 1.8f, 1.6f, 0.0f, 0.02f);
    CHECK(near(s.v, 1.8f));
  }

  // ---- an unclamped S-curve step keeps the planned acceleration bit for bit ----
  {
    tp::AxisState s{.v = 0.3f, .a = 0.7f};
    float planned = s.a;
    tp::update_accel(planned, 1.8f - s.v, 5.0f * 0.02f, 1.0f, 0.02f);
    tp::step_axis(s, 1.8f, -1.8f, 1.8f, 1.0f, 5.0f, 0.02f);
    CHECK(s.a == planned);
  }

  // ---- an S-curve pinned at the velocity bound records no acceleration, and leaves it from 0 ----
  {
    Axis ax{.v_max = 0.5f,
            .a_drive = 1.0f,
            .j_drive = 5.0f,
            .a_stop = 1.6f,
            .j_stop = 5.0f,
            .dt = 0.02f};
    ax.run(0.5f, 100);
    CHECK(near(ax.s.v, 0.5f) && near(ax.s.a, 0.0f));
    ax.step(-0.5f);
    CHECK(near(ax.s.a, -0.1f)); // one jerk step from 0, not from a phantom +1.0
  }

  // ---- the S-curve alone: ramps at the jerk limit, caps at a_max, lands without overshoot ----
  {
    Axis ax{.v_max = 10.0f,
            .a_drive = 2.0f,
            .j_drive = 2.0f,
            .a_stop = 2.0f,
            .j_stop = 2.0f,
            .dt = 0.05f};
    float v_peak = 0.0f, a_peak = 0.0f;
    for (int i = 0; i < 200; i++) {
      ax.step(3.0f);
      v_peak = std::max(v_peak, ax.s.v);
      a_peak = std::max(a_peak, ax.s.a);
    }
    CHECK(near(ax.s.v, 3.0f, 1e-3f) && near(ax.s.a, 0.0f, 1e-3f));
    CHECK(v_peak <= 3.0f + 1e-3f && a_peak <= 2.0f + 1e-6f);
    CHECK(ax.max_jerk_seen <= 2.0f + 1e-3f);
  }

  if (g_failures == 0)
    std::printf("trajectory_planner_host_test: all checks passed\n");
  else
    std::printf("trajectory_planner_host_test: %d check(s) FAILED\n", g_failures);
  return g_failures == 0 ? 0 : 1;
}
