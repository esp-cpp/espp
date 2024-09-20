#pragma once

#include <array>
#include <cmath>

#if defined(ESP_PLATFORM)
#include "esp_dsp.h"
#endif

#include "format.hpp"
#include "transfer_function.hpp"

namespace espp {
/**
 * @brief Digital Biquadratic Filter (Direct Form 1).
 *
 * @note See https://en.wikipedia.org/wiki/Digital_biquad_filter
 *
 * @note See https://www.dsprelated.com/freebooks/filters/Series_Second_Order_Sections.html
 *
 * @note This filter is normalized (meaning the coefficients it uses will have
 *       already been divided by a[0])
 *
 * Implements the following difference equation:
 *
 * @f[
 * y[n] = \frac{b_0 * x[n] + b_1 * x[n-1] + b_2 * x[n-2]
 *            - a_1 * y[n-1] - a_2 * y[n-2]}{a_0}
 * @f]
 */
class BiquadFilterDf1 {
public:
  BiquadFilterDf1() {}

  explicit BiquadFilterDf1(const TransferFunction<3> &tf)
      : BiquadFilterDf1(tf.b, std::array<float, 2>{{tf.a[1], tf.a[2]}}, tf.a[0]) {}

  explicit BiquadFilterDf1(const std::array<float, 3> &b, const std::array<float, 2> &a,
                           const float &a0, const float &gain = 1.0f) {
    b_[0] = b[0] * gain / a0;
    b_[1] = b[1] * gain / a0;
    b_[2] = b[2] * gain / a0;
    // negate the a coefficients so we can use the std::fma which does
    // addition, even though the a coefficients act subtractively.
    a_[0] = -a[0] / a0;
    a_[1] = -a[1] / a0;
  }

  /**
   * @brief Filter the signal sampled by input, updating internal state, and
   *        returning the filtered output.
   * @param input New sample of the input data.
   * @return Filtered output based on input and history.
   */
  float update(float input) {
    float acc = input * b_[0];
    acc = std::fma(prev_x_[0], b_[1], acc);
    acc = std::fma(prev_x_[1], b_[2], acc);
    acc = std::fma(prev_y_[0], a_[0], acc);
    acc = std::fma(prev_y_[1], a_[1], acc);
    prev_x_[1] = prev_x_[0];
    prev_x_[0] = input;
    prev_y_[1] = prev_y_[0];
    prev_y_[0] = acc;
    return prev_y_[0];
  }

  friend struct fmt::formatter<BiquadFilterDf1>;

protected:
  std::array<float, 2> prev_x_ = {0};
  std::array<float, 2> prev_y_ = {0};
  std::array<float, 3> b_ = {0};
  std::array<float, 2> a_ = {0};
};

/**
 * @brief Digital Biquadratic Filter (Direct Form 2). Direct form 2 only needs
 *        N delay units (N is the order), which is potentially half as much as
 *        Direct Form 1, however with Direct Form 2 there is a higher risk of
 *        arithmetic overflow. This implementation uses the esp-dsp
 *        implementation of optimized biquad direct form 2 filter function.
 *
 * @note See https://en.wikipedia.org/wiki/Digital_biquad_filter
 *
 * @note This filter is normalized (meaning the coefficients it uses will have
 *       already been divided by a[0])
 *
 * Implements the following difference equation:
 *
 * @f[
 * y[n] = \frac{b_0 * x[n] + b_1 * x[n-1] + b_2 * x[n-2]
 *            - a_1 * y[n-1] - a_2 * y[n-2]}{a_0}
 * @f]
 */
class BiquadFilterDf2 {
public:
  BiquadFilterDf2() {}

  explicit BiquadFilterDf2(const TransferFunction<3> &tf)
      : BiquadFilterDf2(tf.b, std::array<float, 2>{{tf.a[1], tf.a[2]}}, tf.a[0]) {}

  explicit BiquadFilterDf2(const std::array<float, 3> &b, const std::array<float, 2> &a,
                           const float &a0, const float &gain = 1.0f) {
    b_[0] = b[0] * gain / a0;
    b_[1] = b[1] * gain / a0;
    b_[2] = b[2] * gain / a0;
    a_[0] = a[0] / a0;
    a_[1] = a[1] / a0;
    // coeffs_ used for dsps_biquad_f32 (b0,b1,b2,a1,a2), assumes a0 is 1
    coeffs_[0] = b_[0];
    coeffs_[1] = b_[1];
    coeffs_[2] = b_[2];
    coeffs_[3] = a_[0];
    coeffs_[4] = a_[1];
  }

  /**
   * @brief Filter the signal sampled by input, updating internal state, and
   *        returning the filtered output.
   * @param input Pointer to (floating point) array of new samples of the input data
   * @param output Pointer to (floating point) array which will be filled with
   *        the filtered input.
   * @param length Number of samples, should be >= length of input & output memory.
   */
  void update(const float *input, float *output, size_t length) {
#if defined(ESP_PLATFORM)
    dsps_biquad_f32(input, output, length, coeffs_.data(), w_.data());
#else
    for (size_t i = 0; i < length; i++) {
      output[i] = update(input[i]);
    }
#endif
  }

  /**
   * @brief Filter the signal sampled by input, updating internal state, and
   *        returning the filtered output.
   * @param input New sample of the input data.
   * @return Filtered output based on input and history.
   */
  float update(const float input) {
    float result;
#if defined(ESP_PLATFORM)
    dsps_biquad_f32(&input, &result, 1, coeffs_.data(), w_.data());
#else
    result = input * b_[0] + w_[0];
    w_[0] = input * b_[1] - result * a_[0] + w_[1];
    w_[1] = input * b_[2] - result * a_[1];
#endif
    return result;
  }

  friend struct fmt::formatter<BiquadFilterDf2>;

protected:
  std::array<float, 5> coeffs_ = {{0}};

  std::array<float, 3> b_ = {{0}};
  std::array<float, 2> a_ = {{0}};
  std::array<float, 2> w_ = {{0}};
};
} // namespace espp

#include "biquad_filter_formatters.hpp"
