#pragma once

#include <cmath>

#include "format.hpp"
#include "sos_filter.hpp"

namespace espp {
/**
 * @brief Digital butterworth filter, implemented as biquad sections (Second
 *        Order Sections).
 *
 * @note See https://en.wikipedia.org/wiki/Butterworth_filter for more
 *       information.
 *
 * @tparam ORDER The order of the filter.
 * @tparam Impl Which Biquad implementation form to use.
 */
template <size_t ORDER, class Impl = BiquadFilterDf1>
class ButterworthFilter : public SosFilter<(ORDER + 1) / 2, Impl> {
public:
  /**
   *  @brief Butterworth configuration.
   */
  struct Config {
    float
        normalized_cutoff_frequency; /**< Filter cutoff frequency in the range [0.0, 0.5] (normalizd
                                        to sample frequency, = 2 * f_cutoff / f_sample). */
  };

  /**
   * @brief Construct the butterworth filter for the given config.
   * @param config The configuration struct for the Butterworth Filter
   */
  explicit ButterworthFilter(const Config &config)
      : SosFilter<(ORDER + 1) / 2, Impl>(make_filter_config(config.normalized_cutoff_frequency)) {}

  friend struct fmt::formatter<ButterworthFilter<ORDER, Impl>>;

protected:
  template <size_t N = (ORDER + 1) / 2>
  static std::array<TransferFunction<3>, N> make_filter_config(float normalized_cutoff_frequency) {
    const float gamma = 1.0f / std::tan(M_PI * normalized_cutoff_frequency / 2.0f);
    std::array<TransferFunction<3>, N> filter_config;
    if constexpr (ORDER % 2 == 0) {
      // we have an even order filter
      for (int k = 0; k < N; k++) {
        filter_config[k] = make_sos(k, gamma);
      }
    } else {
      // we have an odd order filter
      for (int k = 0; k < N - 1; k++) {
        filter_config[k] = make_sos(k, gamma);
      }
      filter_config[N - 1] = TransferFunction<3>{
          {{1.0f, 1.0f}},                // b
          {{gamma + 1.0f, 1.0f - gamma}} // a
      };
    }
    return filter_config;
  }

  template <size_t N = (ORDER + 1) / 2> static TransferFunction<3> make_sos(size_t k, float gamma) {
    float g_squared = gamma * gamma;
    float alpha = 2.0f * std::cos(2.0f * M_PI * (2 * k + ORDER + 1) / (4 * ORDER));
    return TransferFunction<3>{{{1.0f, 2.0f, 1.0f}}, // b0, b1, b2
                               {{
                                   g_squared - alpha * gamma + 1, // a0
                                   2.0f * (1.0f - g_squared),     // a1
                                   g_squared + alpha * gamma + 1  // a2
                               }}};
  }
};
} // namespace espp

#include "butterworth_filter_formatters.hpp"
