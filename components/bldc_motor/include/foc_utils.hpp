#pragma once

#include <cmath>

namespace espp {
/**
 * @brief Conversion factor to go from rotations/minute to radians/second.
 */
static constexpr float RPM_TO_RADS = (2.0f * M_PI) / 60.0f;

static constexpr float _2_SQRT3 = 1.15470053838f;
static constexpr float _SQRT3 = 1.73205080757f;
static constexpr float _1_SQRT3 = 0.57735026919f;
static constexpr float _SQRT3_2 = 0.86602540378f;
static constexpr float _SQRT2 = 1.41421356237f;
static constexpr float _120_D2R = 2.09439510239f;
static constexpr float _PI = 3.14159265359f;
static constexpr float _PI_2 = 1.57079632679f;
static constexpr float _PI_3 = 1.0471975512f;
static constexpr float _2PI = 6.28318530718f;
static constexpr float _3PI_2 = 4.71238898038f;
static constexpr float _PI_6 = 0.52359877559f;

static constexpr int _HIGH_IMPEDANCE = 0;
static constexpr int _ACTIVE = 1;

struct DqCurrent {
  float d;
  float q;
};
struct DqVoltage {
  float d;
  float q;
};
struct PhaseCurrent {
  float a;
  float b;
  float c;
};

float normalize_angle(float angle) {
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}

float calc_electrical_angle(float shaft_angle, int pole_pairs) { return shaft_angle * pole_pairs; }
} // namespace espp
