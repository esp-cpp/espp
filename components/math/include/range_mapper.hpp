#pragma once

#include <algorithm>
#include <cmath>

namespace espp {
/**
 * @brief Template class for converting a value from an uncentered [minimum,
 *        maximum] range into a centered output range (default [-1,1]). If
 *        provided a non-zero deadband, it will convert all values within
 *        [center-deadband, center+deadband] to be the configured
 *        output_center (default 0).
 *
 *        The RangeMapper can be optionally configured to invert the input,
 *        so that it will compute the input w.r.t. the configured min/max of
 *        the input range when mapping to the output range - this will mean
 *        that a values within the ranges [minimum, minimum+deadband] and
 *        [maximum-deadband, maximum] will all map to the output_center and
 *        the input center will map to both output_max and output_min
 *        depending on the sign of the input.
 *
 * @tparam T Numeric type to use for the input and output values.
 *
 * @note When inverting the input range, you are introducing a discontinuity
 *       between the input distribution and the output distribution at the
 *       input center. Noise around the input's center value will create
 *       oscillations in the output which will jump between output maximum
 *       and output minimum. Therefore it is advised to use \p invert_input
 *       sparignly, and to set the values robustly.
 *
 *        The RangeMapper can be optionally configured to invert the output,
 *        so that after converting from the input range to the output range,
 *        it will flip the sign on the output.
 *
 * \section range_mapper_ex1 Example
 * \snippet math_example.cpp range_mapper example
 */
template <typename T> class RangeMapper {
public:
  /**
   *  @brief Configuration for the input uncentered range with optional
   *  values for the centered output range, default values of 0 output center
   *  and 1 output range provide a default output range between [-1, 1].
   */
  struct Config {
    T center; /**< Center value for the input range. */
    T center_deadband =
        0;                /**< Deadband amount around (+-) the center for which output will be 0. */
    T minimum;            /**< Minimum value for the input range. */
    T maximum;            /**< Maximum value for the input range. */
    T range_deadband = 0; /**< Deadband amount around the minimum and maximum for which output will
                             be min/max output. */
    T output_center = 0;  /**< The center for the output. Default 0. */
    T output_range = 1;   /**< The range (+/-) from the center for the output. Default 1. @note Will
                             be passed through std::abs() to ensure it is positive. */
    bool invert_output =
        false; /**< Whether to invert the output (default false). @note If true will flip the sign
                  of the output after converting from the input distribution. */
  };

  /**
   * Initialize the range mapper with no config.
   */
  RangeMapper() {}

  /**
   * @brief Initialize the RangeMapper.
   * @param config Configuration describing the input distribution.
   */
  explicit RangeMapper(const Config &config) { configure(config); }

  /**
   * @brief Update the input / output distribution with the new configuration.
   * @param config New configuration to use.
   * @note The output range will be passed through std::abs() to ensure it is
   *       positive.
   * @note The output range must be non-zero. If it is zero, the configuration
   *       will be ignored.
   */
  void configure(const Config &config) {
    center_ = config.center;
    center_deadband_ = std::abs(config.center_deadband);
    minimum_ = config.minimum;
    maximum_ = config.maximum;
    range_deadband_ = std::abs(config.range_deadband);
    output_center_ = config.output_center;
    output_range_ = std::abs(config.output_range);
    output_min_ = output_center_ - output_range_;
    output_max_ = output_center_ + output_range_;
    // positive range is the range from the (center + center_deadband) to (max - range_deadband)
    pos_range_ = output_range_
                     ? (maximum_ - range_deadband_ - (center_ + center_deadband_)) / output_range_
                     : 0;
    // negative range is the range from the (center - center_deadband) to (min + range_deadband)
    neg_range_ = output_range_
                     ? (center_ - center_deadband_ - (minimum_ + range_deadband_)) / output_range_
                     : 0;
    invert_output_ = config.invert_output;
  }

  /**
   * @brief Return the configured center of the input distribution
   * @return Center of the input distribution for this range mapper.
   */
  T get_center() const { return center_; }

  /**
   * @brief Return the configured deadband around the center of the input
   *        distribution
   * @return Deadband around the center of the input distribution for this
   *         range mapper.
   */
  T get_center_deadband() const { return center_deadband_; }

  /**
   * @brief Return the configured minimum of the input distribution
   * @return Minimum of the input distribution for this range mapper.
   */
  T get_minimum() const { return minimum_; }

  /**
   * @brief Return the configured maximum of the input distribution
   * @return Maximum of the input distribution for this range mapper.
   */
  T get_maximum() const { return maximum_; }

  /**
   * @brief Return the configured range of the input distribution
   * @note Always positive.
   * @return Range of the input distribution for this range mapper.
   */
  T get_range() const { return maximum_ - minimum_; }

  /**
   * @brief Return the configured deadband around the min/max of the input
   *        distribution
   * @return Deadband around the min/max of the input distribution for this
   *         range mapper.
   */
  T get_range_deadband() const { return range_deadband_; }

  /**
   * @brief Return the configured center of the output distribution
   * @return Center of the output distribution for this range mapper.
   */
  T get_output_center() const { return output_center_; }

  /**
   * @brief Return the configured range of the output distribution
   * @note Always positive.
   * @return Range of the output distribution for this range mapper.
   */
  T get_output_range() const { return output_range_; }

  /**
   * @brief Return the configured minimum of the output distribution
   * @return Minimum of the output distribution for this range mapper.
   */
  T get_output_min() const { return output_min_; }

  /**
   * @brief Return the configured maximum of the output distribution
   * @return Maximum of the output distribution for this range mapper.
   */
  T get_output_max() const { return output_max_; }

  /**
   * @brief Set the deadband around the center of the input distribution.
   * @param deadband The deadband to use around the center of the input
   *        distribution.
   * @note The deadband must be non-negative.
   * @note The deadband is applied around the center value of the input
   *       distribution.
   */
  void set_center_deadband(T deadband) { center_deadband_ = std::abs(deadband); }

  /**
   * @brief Set the deadband around the min/max of the input distribution.
   * @param deadband The deadband to use around the min/max of the input
   *        distribution.
   * @note The deadband must be non-negative.
   * @note The deadband is applied around the min/max values of the input
   *       distribution.
   */
  void set_range_deadband(T deadband) { range_deadband_ = std::abs(deadband); }

  /**
   * @brief Map a value \p v from the input distribution into the configured
   *        output range (centered, default [-1,1]).
   * @param v Value from the (possibly uncentered and possibly inverted -
   *        defined by the previously configured Config) input distribution
   * @return Value within the centered output distribution.
   */
  T map(const T &v) const {
    T clamped = std::clamp(v, minimum_, maximum_);
    T calibrated{0};
    // compare against center
    calibrated = clamped - center_;
    bool min_eq_center = minimum_ == center_;
    bool max_eq_center = maximum_ == center_;
    bool within_center_deadband = std::abs(calibrated) < center_deadband_;
    // Ensure we handle the case that the center is equal to the min or max,
    // which may happen if the user wants a uni-directional output (e.g. [0,1]
    // instead of [-1,1]). In this case, we only apply the center deadband, not
    // the range deadband.
    bool within_max_deadband = !max_eq_center && clamped >= (maximum_ - range_deadband_);
    bool within_min_deadband = !min_eq_center && clamped <= (minimum_ + range_deadband_);
    if (within_center_deadband) {
      return output_center_;
    } else if (within_min_deadband) {
      return invert_output_ ? output_max_ : output_min_;
    } else if (within_max_deadband) {
      return invert_output_ ? output_min_ : output_max_;
    }

    bool positive_input = calibrated >= 0;
    // remove the deadband from the calibrated value
    calibrated = positive_input ? calibrated - center_deadband_ : calibrated + center_deadband_;

    T output = positive_input ? calibrated / pos_range_ : calibrated / neg_range_;
    if (invert_output_) {
      output = -output;
    }
    output += output_center_;
    return std::clamp(output, output_min_, output_max_);
  }

  /**
   * @brief Unmap a value \p v from the configured output range (centered,
   *        default [-1,1]) back into the input distribution.
   * @param v Value from the centered output distribution.
   * @return Value within the input distribution.
   */
  T unmap(const T &v) const {
    T clamped = std::clamp(v, output_min_, output_max_);
    // return early if we're in the center deadband
    if (clamped == output_center_) {
      return center_;
    }
    // return early if we're in the range deadband
    if (clamped == output_min_ || clamped == output_max_) {
      return invert_output_           ? clamped == output_min_ ? (maximum_ - range_deadband_)
                                                               : (minimum_ + range_deadband_)
                       : clamped == output_min_ ? (minimum_ + range_deadband_)
                                      : (maximum_ - range_deadband_);
    }
    // else we need to convert the output value back to the input range
    if (invert_output_) {
      // if the output is inverted, we need to invert the output value (flip
      // around the output center)
      clamped = output_center_ - (clamped - output_center_);
    }
    bool positive_output = clamped >= output_center_;
    T calibrated{0};
    if (positive_output) {
      calibrated = (clamped - output_center_) * pos_range_ + center_ + center_deadband_;
    } else {
      calibrated = (clamped - output_center_) * neg_range_ + center_ - center_deadband_;
    }
    return std::clamp(calibrated, minimum_, maximum_);
  }

protected:
  T center_{0};
  T center_deadband_{0};
  T minimum_{0};
  T maximum_{0};
  T range_deadband_{0};
  T pos_range_{1};
  T neg_range_{1};
  T output_center_{0};
  T output_range_{0};
  T output_min_{0};
  T output_max_{0};
  bool invert_output_{false};
};

/**
 * @brief Typedef for float ranges.
 */
typedef RangeMapper<float> FloatRangeMapper;

/**
 * @brief Typedef for int ranges.
 */
typedef RangeMapper<int> IntRangeMapper;
} // namespace espp

#include "range_mapper_formatters.hpp"
