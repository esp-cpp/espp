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
    */
  template<typename T> class RangeMapper {
  public:
    /**
      *  @brief Configuration for the input uncentered range with optional
      *  values for the centered output range, default values of 0 output center
      *  and 1 output range provide a default output range between [-1, 1].
      */
    struct Config {
      T center;  /**< Center value for the input range. */
      T deadband;/**< Deadband amount around (+-) the center for which output will be 0. */
      T minimum; /**< Minimum value for the input range. */
      T maximum; /**< Maximum value for the input range. */
      T output_center{T(0)}; /**< The center for the output. Default 0. */
      T output_range{T(1)}; /**< The range +/- from the center for the output. Default +/- 1. */
    };

    /**
     * @brief Initialize the RangeMapper.
     * @param config Configuration describing the input distribution.
     */
    RangeMapper(const Config& config) {
      configure(config);
    }

    /**
     * @brief Update the input / output distribution with the new configuration.
     * @param config New configuration to use.
     */
    void configure(const Config& config) {
      center_ = config.center;
      deadband_ = config.deadband;
      minimum_ = config.minimum;
      maximum_ = config.maximum;
      output_center_ = config.output_center;
      output_range_ = config.output_range;
      pos_range_ = (maximum_ - center_) / output_range_;
      neg_range_ = std::abs(minimum_ - center_) / output_range_;
    }

    /**
     * @brief Map a value \p v from the input distribution into the configured
     *        output range (centered, default [-1,1]).
     * @param v Value from the (possibly uncentered) input distribution
     * @return Value within the centered output distribution.
     */
    T map(const T& v) {
      T calibrated = std::clamp(v, minimum_, maximum_) - center_;
      if (std::abs(calibrated) < deadband_) {
        return output_center_;
      }
      return (calibrated > T(0)) ?
        calibrated / pos_range_ + output_center_ : calibrated / neg_range_ + output_center_;
    }

  protected:
    T center_;
    T deadband_;
    T minimum_;
    T maximum_;
    T pos_range_;
    T neg_range_;
    T output_center_;
    T output_range_;
  };

  /**
   * @brief Typedef for float ranges.
   */
  typedef RangeMapper<float> FloatRangeMapper;

  /**
   * @brief Typedef for int ranges.
   */
  typedef RangeMapper<int> IntRangeMapper;
}
