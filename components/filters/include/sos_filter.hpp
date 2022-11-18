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
  template <size_t N, class SectionImpl = BiquadFilterDf2>
  class SosFilter {
  public:
    /**
     * @brief Construct a second order sections filter.
     * @param config Array of TransferFunction<3> for configuring each of the biquad sections.
     */
    SosFilter(const std::array<TransferFunction<3>, N>& config) {
      for (int i = 0; i<N; i++) {
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
     * @brief Format all the sections' coefficients to string and return it.
     * @return std::string of formatted section coefficients, newline separated.
     */
    std::string to_string() {
      std::string ret;
      for (auto section : sections_)
        ret += section.to_string() + "\n";
      return ret;
    }

  protected:
    std::array<SectionImpl, N> sections_;
  };
}
