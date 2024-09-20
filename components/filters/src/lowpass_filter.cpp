#include "lowpass_filter.hpp"

using namespace espp;

LowpassFilter::LowpassFilter(const Config &config) { init(config); }

void LowpassFilter::update(const float *input, float *output, size_t length) {
#if defined(ESP_PLATFORM)
  dsps_biquad_f32(input, output, length, coeffs_, state_);
#else
  for (size_t i = 0; i < length; i++) {
    output[i] = update(input[i]);
  }
#endif
}

void LowpassFilter::configure(const Config &config) { init(config); }

float LowpassFilter::update(const float input) {
  float output;
#if defined(ESP_PLATFORM)
  dsps_biquad_f32(&input, &output, 1, coeffs_, state_);
#else
  // shift the input into the state buffer
  state_[4] = state_[3];
  state_[3] = state_[2];
  state_[2] = state_[1];
  state_[1] = state_[0];
  state_[0] = input;
  // calculate the output
  output = coeffs_[0] * state_[0] + coeffs_[1] * state_[1] + coeffs_[2] * state_[2] +
           coeffs_[3] * state_[3] + coeffs_[4] * state_[4];
#endif
  return output;
}

float LowpassFilter::operator()(float input) { return update(input); }

void LowpassFilter::init(const Config &config) {
#if defined(ESP_PLATFORM)
  dsps_biquad_gen_lpf_f32(coeffs_, config.normalized_cutoff_frequency, config.q_factor);
#else
  float w0 = 2 * M_PI * config.normalized_cutoff_frequency;
  float alpha = sin(w0) / (2 * config.q_factor);
  float cos_w0 = cos(w0);
  float b0 = (1 - cos_w0) / 2;
  float b1 = 1 - cos_w0;
  float b2 = (1 - cos_w0) / 2;
  float a0 = 1 + alpha;
  float a1 = -2 * cos_w0;
  float a2 = 1 - alpha;
  coeffs_[0] = b0 / a0;
  coeffs_[1] = b1 / a0;
  coeffs_[2] = b2 / a0;
  coeffs_[3] = a1 / a0;
  coeffs_[4] = a2 / a0;
#endif
}

void LowpassFilter::reset() {
  for (size_t i = 0; i < 5; i++) {
    state_[i] = 0;
  }
}
