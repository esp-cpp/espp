#include "simple_lowpass_filter.hpp"

using namespace espp;

SimpleLowpassFilter::SimpleLowpassFilter(const Config &config)
    : time_constant_(config.time_constant) {}

void SimpleLowpassFilter::set_time_constant(const float time_constant) {
  time_constant_ = time_constant;
}

float SimpleLowpassFilter::get_time_constant() const { return time_constant_; }

float SimpleLowpassFilter::update(const float input) {
  float output;
#if defined(ESP_PLATFORM)
  uint64_t time = esp_timer_get_time();
  float dt = (time - prev_time_) / 1e6f;
  prev_time_ = time;
#else
  auto time = std::chrono::high_resolution_clock::now();
  float dt = std::chrono::duration<float>(time - prev_time_).count();
  prev_time_ = time;
#endif
  if (dt <= 0) {
    return prev_output_;
  }
  float alpha = dt / (time_constant_ + dt);
  output = alpha * input + (1.0f - alpha) * prev_output_;
  prev_output_ = output;
  return output;
}

float SimpleLowpassFilter::operator()(float input) { return update(input); }

void SimpleLowpassFilter::reset() {
  prev_output_ = 0;
#if defined(ESP_PLATFORM)
  prev_time_ = esp_timer_get_time();
#else
  prev_time_ = std::chrono::high_resolution_clock::now();
#endif
}
