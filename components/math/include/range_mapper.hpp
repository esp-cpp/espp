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
 */
template <typename T> class RangeMapper {
public:
  /**
   *  @brief Configuration for the input uncentered range with optional
   *  values for the centered output range, default values of 0 output center
   *  and 1 output range provide a default output range between [-1, 1].
   */
  struct Config {
    T center;   /**< Center value for the input range. */
    T deadband; /**< Deadband amount around (+-) the center for which output will be 0. */
    T minimum;  /**< Minimum value for the input range. */
    T maximum;  /**< Maximum value for the input range. */
    bool invert_input{
        false}; /**< Whether to invert the input distribution (default false). @note If true will
                   compute the input relative to min/max instead of to center. */
    T output_center{T(0)}; /**< The center for the output. Default 0. */
    T output_range{T(1)}; /**< The range (+/-) from the center for the output. Default 1. @note Will
                             be passed through std::abs() to ensure it is positive. */
    bool invert_output{
        false}; /**< Whether to invert the output (default false). @note If true will flip the sign
                   of the output after converting from the input distribution. */
  };

  /**
   * Initialize the range mapper with no config.
   */
  RangeMapper() = default;

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
    if (config.output_range == T(0)) {
      return;
    }

    center_ = config.center;
    deadband_ = config.deadband;
    minimum_ = config.minimum;
    maximum_ = config.maximum;
    invert_input_ = config.invert_input;
    output_center_ = config.output_center;
    output_range_ = std::abs(config.output_range);
    output_min_ = output_center_ - output_range_;
    output_max_ = output_center_ + output_range_;
    pos_range_ = (maximum_ - center_) / output_range_;
    neg_range_ = std::abs(minimum_ - center_) / output_range_;
    invert_output_ = config.invert_output;
  }

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
   * @brief Set the deadband for the input distribution.
   * @param deadband The deadband to use for the input distribution.
   * @note The deadband must be non-negative.
   * @note The deadband is applied around the center value of the input
   *       distribution.
   */
  void set_deadband(T deadband) { deadband_ = deadband; }

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
    if (invert_input_) {
      // if we invert the input, then we are comparing against the min/max
      calibrated = clamped >= T(0) ? maximum_ - clamped : minimum_ - clamped;
    } else {
      // normally we compare against center
      calibrated = clamped - center_;
    }
    if (std::abs(calibrated) < deadband_) {
      return output_center_;
    }
    T output = calibrated >= T(0) ? calibrated / pos_range_ + output_center_
                                  : calibrated / neg_range_ + output_center_;
    if (invert_output_) {
      output = -output;
    }
    return output;
  }

  /**
   * @brief Unmap a value \p v from the configured output range (centered,
   *        default [-1,1]) back into the input distribution.
   * @param T&v Value from the centered output distribution.
   * @return Value within the input distribution.
   */
  T unmap(const T &v) const {
    T calibrated =
        v >= T(0) ? (v - output_center_) * pos_range_ : (v - output_center_) * neg_range_;
    if (invert_output_) {
      calibrated = -calibrated;
    }
    T clamped = calibrated + center_;
    if (invert_input_) {
      clamped = calibrated >= T(0) ? maximum_ - calibrated : minimum_ - calibrated;
    }
    return std::clamp(clamped, minimum_, maximum_);
  }

protected:
  T center_{0};
  T deadband_{0};
  T minimum_{0};
  T maximum_{0};
  bool invert_input_{false};
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

#include "fmt/format.h"

// NOTE: right now it seems we cannot use the generic version that works in
//       vector2d.hpp because it results in 'template class without a name'
//       errors...

template <> struct fmt::formatter<espp::FloatRangeMapper::Config> : fmt::formatter<std::string> {
  auto format(const espp::FloatRangeMapper::Config &config, format_context &ctx) {
    return fmt::format_to(ctx.out(), "[{},{},{},{},{},{},{},{}]", config.center, config.deadband,
                          config.minimum, config.maximum, config.invert_input, config.output_center,
                          config.output_range, config.invert_output);
  }
};
template <> struct fmt::formatter<espp::IntRangeMapper::Config> : fmt::formatter<std::string> {
  auto format(const espp::IntRangeMapper::Config &config, format_context &ctx) {
    return fmt::format_to(ctx.out(), "[{},{},{},{},{},{},{},{}]", config.center, config.deadband,
                          config.minimum, config.maximum, config.invert_input, config.output_center,
                          config.output_range, config.invert_output);
  }
};
