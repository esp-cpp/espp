#pragma once

#include "format.hpp"

#include "biquad_filter.hpp"

namespace espp {
/**
 * @brief Second Order Sections Filter.
 *
 * @note See https://en.wikipedia.org/wiki/Digital_biquad_filter
 *
 * @note See https://www.dsprelated.com/freebooks/filters/Series_Second_Order_Sections.html
 */
template <size_t N, class SectionImpl = BiquadFilterDf2> class SosFilter {
public:
  /**
   * @brief Construct a second order sections filter.
   * @param config Array of TransferFunction<3> for configuring each of the biquad sections.
   */
  explicit SosFilter(const std::array<TransferFunction<3>, N> &config) {
    for (int i = 0; i < N; i++) {
      sections_[i] = SectionImpl(config[i]);
    }
  }

  /**
   * @brief Filter the signal sampled by input, updating internal state, and
   *        returning the filtered output.
   * @param input New sample of the input data.
   * @return Filtered output based on input and history.
   */
  float update(float input) {
    float output = input;
    // pass the input through each section, in order
    for (auto &section : sections_) {
      output = section.update(output);
    }
    return output;
  }

  /**
   * @brief Filter the signal sampled by input, updating internal state, and
   *        returning the filtered output.
   * @param input New sample of the input data.
   * @return Filtered output based on input and history.
   */
  float operator()(float input) { return update(input); }

  friend struct fmt::formatter<SosFilter<N, SectionImpl>>;

protected:
  std::array<SectionImpl, N> sections_;
};
} // namespace espp

// for allowing easy serialization/printing of the
// espp::SosFilter
template <size_t N, class SectionImpl> struct fmt::formatter<espp::SosFilter<N, SectionImpl>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::SosFilter<N, SectionImpl> const &f, FormatContext &ctx) const {
    auto &&out = ctx.out();
    format_to(out, "SoS - [");
    if constexpr (N > 0) {
      format_to(out, "[{}]", f.sections_[0]);
    }
    for (int i = 1; i < N; i++) {
      format_to(out, ", [{}]", f.sections_[i]);
    }
    return fmt::format_to(out, "]");
  }
};
