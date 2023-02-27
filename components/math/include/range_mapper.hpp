#pragma once

#include <cmath>

namespace espp {
  /**
    * @brief Template class for converting a value from an uncentered [minimum,
    *        maximum] range into a centered [-1,1] range. If provided a non-zero
    *        deadband, it will convert all values within [center-deadband,
    *        center+deadband] to be 0.
    */
  template<typename T> class RangeMapper {
  public:
    /**
      *  @brief Configuration for the input uncentered range.
      */
    struct Config {
      T center;  /**< Center value for the input range. */
      T deadband;/**< Deadband amount around (+-) the center for which output will be 0. */
      T minimum; /**< Minimum value for the input range. */
      T maximum; /**< Maximum value for the input range. */
      T output_center{T(0)}; /**< The center for the output. */
      T output_range{T(1)}; /**< The range +/- from the center for the output. */
    };

    /**
     * @brief Initialize the RangeMapper.
     * @param config Configuration describing the input distribution.
     */
    RangeMapper(const Config& config) {
      configure(config);
    }

    /**
     * @brief Update the input distribution with the new configuration.
     * @param config New input distribution configuration to use.
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
     * @brief Map a value \p v from the input distribution into the range
     *        [-1,1].
     * @param v Value from the input distribution
     * @return Value within range [-1,1]
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
