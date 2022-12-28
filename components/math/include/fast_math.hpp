#pragma once

#include <cstdlib>
#include <cmath>
#include <cstring>

namespace espp {
  /**
   * @brief Simple square of the input.
   * @param f Value to square.
   * @return The square of f (f*f).
   */
  float square(float f) {
    return f * f;
  }

  /**
   * @brief Simple cube of the input.
   * @param f Value to cube.
   * @return The cube of f (f*f*f).
   */
  float cube(float f) {
    return f * f * f;
  }

  /**
   * @brief Fast square root approximation.
   * @note Using https://reprap.org/forum/read.php?147,219210 and
   *       https://en.wikipedia.org/wiki/Fast_inverse_square_root
   * @param value [description]
   * @return [description]
   */
  float fast_sqrt(float value) {
    uint32_t i{0};
    float y{0};
    // float x;
    // const float f = 1.5F; // better precision

    // x = number * 0.5F;
    y = value;
    memcpy(&i, &y, sizeof(i));
    // i = * reinterpret_cast<uint32_t *>(&y);
    i = 0x5f375a86 - ( i >> 1 );
    // y = * reinterpret_cast<float *>(&i);
    memcpy(&y, &i, sizeof(y));
    // y = y * ( f - ( x * y * y ) ); // better precision
    return value * y;
  }

  /**
   * @brief Get the sign of a number (+1, 0, or -1)
   * @param x Value to get the sign of
   * @return Sign of x: -1 if x < 0, 0 if x == 0, or +1 if x > 0
   */
  template <typename T>
  int sgn(T x) {
    return (T(0) < x) - (x < T(0));
  }

  /**
   * @brief Round x to the nearest integer.
   * @param x Floating point value to be rounded.
   * @return Nearest integer to x.
   */
  int round(float x) {
    return (x > 0) ? (int)(x + 0.5) : (int)(x - 0.5);
  }

  /**
   * @brief fast natural log function, ln(x).
   * @note This speed hack comes from:
   *   https://gist.github.com/LingDong-/7e4c4cae5cbbc44400a05fba65f06f23
   * @param x Value to take the natural log of.
   * @return ln(x)
   */
  float fast_ln(float x) {
    uint32_t bx = 0;
    memcpy(&bx, &x, sizeof(bx));
    // bx = * reinterpret_cast<uint32_t *> (&x);
    uint32_t ex = bx >> 23;
    signed int t = (signed int)ex-(signed int)127;
    uint32_t s = (t < 0) ? (-t) : t;
    bx = 1065353216 | (bx & 8388607);
    // x = * reinterpret_cast<float *>(&bx);
    memcpy (&x, &bx, sizeof(x));
    return -1.49278+(2.11263+(-0.729104+0.10969*x)*x)*x+0.6931471806*t;
  }

  /**
   *  @brief Int lookup table for sin, values represent the first quadrant of
   *         sin(x) * 10000.
   */
  static constexpr int sine_array[200] = {
    0,    79,   158,  237,  316,  395,  473,   552,  631,  710,  789,  867,
    946,  1024, 1103, 1181, 1260, 1338, 1416,  1494, 1572, 1650, 1728, 1806,
    1883, 1961, 2038, 2115, 2192, 2269, 2346,  2423, 2499, 2575, 2652, 2728,
    2804, 2879, 2955, 3030, 3105, 3180, 3255,  3329, 3404, 3478, 3552, 3625,
    3699, 3772, 3845, 3918, 3990, 4063, 4135,  4206, 4278, 4349, 4420, 4491,
    4561, 4631, 4701, 4770, 4840, 4909, 4977,  5046, 5113, 5181, 5249, 5316,
    5382, 5449, 5515, 5580, 5646, 5711, 5775,  5839, 5903, 5967, 6030, 6093,
    6155, 6217, 6279, 6340, 6401, 6461, 6521,  6581, 6640, 6699, 6758, 6815,
    6873, 6930, 6987, 7043, 7099, 7154, 7209,  7264, 7318, 7371, 7424, 7477,
    7529, 7581, 7632, 7683, 7733, 7783, 7832,  7881, 7930, 7977, 8025, 8072,
    8118, 8164, 8209, 8254, 8298, 8342, 8385,  8428, 8470, 8512, 8553, 8594,
    8634, 8673, 8712, 8751, 8789, 8826, 8863,  8899, 8935, 8970, 9005, 9039,
    9072, 9105, 9138, 9169, 9201, 9231, 9261,  9291, 9320, 9348, 9376, 9403,
    9429, 9455, 9481, 9506, 9530, 9554, 9577,  9599, 9621, 9642, 9663, 9683,
    9702, 9721, 9739, 9757, 9774, 9790, 9806,  9821, 9836, 9850, 9863, 9876,
    9888, 9899, 9910, 9920, 9930, 9939, 9947,  9955, 9962, 9969, 9975, 9980,
    9985, 9989, 9992, 9995, 9997, 9999, 10000, 10000
  };

  /**
   * @brief Fast approximation of sin(angle) (radians).
   * @note \p Angle must be in the range [0, 2PI].
   * @param angle Angle in radians [0, 2*PI]
   * @return Approximation of sin(value)
   */
  float fast_sin(float angle) {
    if (angle < M_PI_2) {
      return 0.0001f * sine_array[round(126.6873f * angle)];
    } else if (angle < M_PI) {
      return 0.0001f * sine_array[398 - round(126.6873f * angle)];
    } else if (angle < (3.0f * M_PI_2)) {
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
  float fast_cos(float angle) {
    float a_sin = angle + M_PI_2;
    a_sin = (a_sin > (2.0f * M_PI)) ? (a_sin - (2.0f * M_PI)) : a_sin;
    return fast_sin(a_sin);
  }
}
