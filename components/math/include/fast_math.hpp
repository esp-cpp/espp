#pragma once

#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <utility>
#include <vector>

namespace espp {
/**
 * @brief Convert degrees to radians
 * @param degrees Angle in degrees
 * @return Angle in radians
 */
[[maybe_unused]] static float deg_to_rad(float degrees) { return degrees * (float)(M_PI) / 180.0f; }

/**
 * @brief Convert radians to degrees
 * @param radians Angle in radians
 * @return Angle in degrees
 */
[[maybe_unused]] static float rad_to_deg(float radians) { return radians * 180.0f / (float)(M_PI); }

/**
 * @brief Simple square of the input.
 * @param f Value to square.
 * @return The square of f (f*f).
 */
[[maybe_unused]] static float square(float f) { return f * f; }

/**
 * @brief Simple cube of the input.
 * @param f Value to cube.
 * @return The cube of f (f*f*f).
 */
[[maybe_unused]] static float cube(float f) { return f * f * f; }

/**
 * @brief Fast inverse square root approximation.
 * @note Using https://reprap.org/forum/read.php?147,219210 and
 *       https://en.wikipedia.org/wiki/Fast_inverse_square_root
 * @param value Value to take the inverse square root of.
 * @return Approximation of the inverse square root of value.
 */
[[maybe_unused]] static constexpr float fast_inv_sqrt(float value) noexcept {
  const auto y = std::bit_cast<float>(0x5f3759df - (std::bit_cast<std::uint32_t>(value) >> 1));
  return y * (1.5f - (0.5f * value * y * y));
}

/**
 * @brief Get the sign of a number (+1, 0, or -1)
 * @param x Value to get the sign of
 * @return Sign of x: -1 if x < 0, 0 if x == 0, or +1 if x > 0
 */
template <typename T> int sgn(T x) { return (T(0) < x) - (x < T(0)); }

/**
 * @brief Linear interpolation between two values.
 * @param a First value.
 * @param b Second value.
 * @param t Interpolation factor in the range [0, 1].
 * @return Linear interpolation between a and b.
 */
[[maybe_unused]] static float lerp(float a, float b, float t) { return a + t * (b - a); }

/**
 * @brief Compute the inverse lerped value.
 * @param a First value (usually the lower of the two).
 * @param b Second value (usually the higher of the two).
 * @param v Value to inverse lerp (usually a value between a and b).
 * @return Inverse lerp value, the factor of v between a and b in the range [0,
 *         1] if v is between a and b, 0 if v == a, or 1 if v == b. If a == b,
 *         0 is returned. If v is outside the range [a, b], the value is
 *         extrapolated linearly (i.e. if v < a, the value is less than 0, if v
 *         > b, the value is greater than 1).
 */
[[maybe_unused]] static float inv_lerp(float a, float b, float v) {
  if (a == b) {
    return 0.0f;
  }
  return (v - a) / (b - a);
}

/**
 * @brief Compute the piecewise linear interpolation between a set of points.
 * @param points Vector of points to interpolate between. The vector should be
 *               sorted by the first value in the pair. The first value in the
 *               pair is the x value and the second value is the y value. The x
 *               values should be unique. The function will interpolate between
 *               the points using linear interpolation. If x is less than the
 *               first x value, the first y value is returned. If x is greater
 *               than the last x value, the last y value is returned. If x is
 *               between two x values, the y value is interpolated between the
 *               two y values.
 * @param x Value to interpolate at. Should be a value from the first
 *          distribution of the points (the domain). If x is outside the domain
 *          of the points, the value returned will be clamped to the first or
 *          last y value.
 * @return Interpolated value at x.
 */
[[maybe_unused]] static float piecewise_linear(const std::vector<std::pair<float, float>> &points,
                                               float x) {
  if (points.size() == 0) {
    return 0.0f;
  }
  if (x <= points[0].first) {
    return points[0].second;
  }
  if (x >= points[points.size() - 1].first) {
    return points[points.size() - 1].second;
  }
  for (size_t i = 1; i < points.size(); i++) {
    if (x <= points[i].first) {
      float t = inv_lerp(points[i - 1].first, points[i].first, x);
      return lerp(points[i - 1].second, points[i].second, t);
    }
  }
  return 0.0f;
}

/**
 * @brief Round x to the nearest integer.
 * @param x Floating point value to be rounded.
 * @return Nearest integer to x.
 */
[[maybe_unused]] static int round(float x) { return (x > 0) ? (int)(x + 0.5f) : (int)(x - 0.5f); }

/**
 * @brief fast natural log function, ln(x).
 * @note This speed hack comes from:
 *   https://gist.github.com/LingDong-/7e4c4cae5cbbc44400a05fba65f06f23
 * @param x Value to take the natural log of.
 * @return ln(x)
 */
[[maybe_unused]] static float fast_ln(float x) {
  uint32_t bx = 0;
  memcpy(&bx, &x, sizeof(bx));
  // bx = * reinterpret_cast<uint32_t *> (&x);
  uint32_t ex = bx >> 23;
  signed int t = (signed int)ex - (signed int)127;
  [[maybe_unused]] uint32_t s = (t < 0) ? (-t) : t;
  bx = 1065353216 | (bx & 8388607);
  // x = * reinterpret_cast<float *>(&bx);
  memcpy(&x, &bx, sizeof(x));
  return -1.49278f + (2.11263f + (-0.729104f + 0.10969f * x) * x) * x + 0.6931471806f * t;
}

/**
 *  @brief Int lookup table for sin, values represent the first quadrant of
 *         sin(x) * 10000.
 */
static constexpr int sine_array[200] = {
    0,    79,   158,  237,  316,  395,  473,   552,  631,  710,  789,  867,  946,  1024, 1103, 1181,
    1260, 1338, 1416, 1494, 1572, 1650, 1728,  1806, 1883, 1961, 2038, 2115, 2192, 2269, 2346, 2423,
    2499, 2575, 2652, 2728, 2804, 2879, 2955,  3030, 3105, 3180, 3255, 3329, 3404, 3478, 3552, 3625,
    3699, 3772, 3845, 3918, 3990, 4063, 4135,  4206, 4278, 4349, 4420, 4491, 4561, 4631, 4701, 4770,
    4840, 4909, 4977, 5046, 5113, 5181, 5249,  5316, 5382, 5449, 5515, 5580, 5646, 5711, 5775, 5839,
    5903, 5967, 6030, 6093, 6155, 6217, 6279,  6340, 6401, 6461, 6521, 6581, 6640, 6699, 6758, 6815,
    6873, 6930, 6987, 7043, 7099, 7154, 7209,  7264, 7318, 7371, 7424, 7477, 7529, 7581, 7632, 7683,
    7733, 7783, 7832, 7881, 7930, 7977, 8025,  8072, 8118, 8164, 8209, 8254, 8298, 8342, 8385, 8428,
    8470, 8512, 8553, 8594, 8634, 8673, 8712,  8751, 8789, 8826, 8863, 8899, 8935, 8970, 9005, 9039,
    9072, 9105, 9138, 9169, 9201, 9231, 9261,  9291, 9320, 9348, 9376, 9403, 9429, 9455, 9481, 9506,
    9530, 9554, 9577, 9599, 9621, 9642, 9663,  9683, 9702, 9721, 9739, 9757, 9774, 9790, 9806, 9821,
    9836, 9850, 9863, 9876, 9888, 9899, 9910,  9920, 9930, 9939, 9947, 9955, 9962, 9969, 9975, 9980,
    9985, 9989, 9992, 9995, 9997, 9999, 10000, 10000};

/**
 * @brief Fast approximation of sin(angle) (radians).
 * @note \p Angle must be in the range [0, 2PI].
 * @param angle Angle in radians [0, 2*PI]
 * @return Approximation of sin(value)
 */
[[maybe_unused]] static float fast_sin(float angle) {
  if (angle < (float)(M_PI_2)) {
    return 0.0001f * sine_array[round(126.6873f * angle)];
  } else if (angle < (float)(M_PI)) {
    return 0.0001f * sine_array[398 - round(126.6873f * angle)];
  } else if (angle < (3.0f * (float)(M_PI_2))) {
    return -0.0001f * sine_array[round(126.6873f * angle) - 398];
  } else {
    return -0.0001f * sine_array[796 - round(126.6873f * angle)];
  }
}

/**
 * @brief Fast approximation of cos(angle) (radians).
 * @note \p Angle must be in the range [0, 2PI].
 * @param angle Angle in radians [0, 2*PI]
 * @return Approximation of cos(value)
 */
[[maybe_unused]] static float fast_cos(float angle) {
  float a_sin = angle + (float)(M_PI_2);
  a_sin = (a_sin > (2.0f * (float)(M_PI))) ? (a_sin - (2.0f * (float)(M_PI))) : a_sin;
  return fast_sin(a_sin);
}
} // namespace espp
