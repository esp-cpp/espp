#include "bdc_driver.hpp"

#include <algorithm>

using namespace espp;

BdcDriver::BdcDriver(const Config &config)
    : BaseComponent("BdcDriver", config.log_level)
    , gpio_a_(config.gpio_a)
    , gpio_b_(config.gpio_b)
    , gpio_enable_(config.gpio_enable)
    , group_id_(config.group_id)
    , timer_resolution_hz_(config.timer_resolution_hz)
    , pwm_frequency_hz_(config.pwm_frequency_hz) {
  initialized_ = init(config);
  if (!initialized_) {
    cleanup();
  }
}

BdcDriver::~BdcDriver() { cleanup(); }

bool BdcDriver::initialized() const { return initialized_; }

bool BdcDriver::enable() {
  if (!initialized_) {
    logger_.error("Cannot enable brushed motor driver before initialization");
    return false;
  }
  if (enabled_) {
    return true;
  }
  esp_err_t err = mcpwm_timer_enable(timer_);
  if (err != ESP_OK) {
    logger_.error("Failed to enable MCPWM timer: {}", esp_err_to_name(err));
    return false;
  }
  err = mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_NO_STOP);
  if (err != ESP_OK) {
    logger_.error("Failed to start MCPWM timer: {}", esp_err_to_name(err));
    mcpwm_timer_disable(timer_);
    return false;
  }
  if (gpio_enable_ != GPIO_NUM_NC) {
    gpio_set_level(gpio_enable_, 1);
  }
  enabled_ = true;
  return true;
}

bool BdcDriver::disable() {
  if (!timer_) {
    return true;
  }
  bool ok = true;
  if (gpio_enable_ != GPIO_NUM_NC) {
    gpio_set_level(gpio_enable_, 0);
  }
  for (auto &generator : generators_) {
    if (!generator) {
      continue;
    }
    esp_err_t err = mcpwm_generator_set_force_level(generator, 0, true);
    if (err != ESP_OK) {
      logger_.error("Failed to force MCPWM output low during disable: {}", esp_err_to_name(err));
      ok = false;
    }
  }
  if (enabled_) {
    esp_err_t err = mcpwm_timer_start_stop(timer_, MCPWM_TIMER_STOP_FULL);
    if (err != ESP_OK) {
      logger_.error("Failed to stop MCPWM timer: {}", esp_err_to_name(err));
      ok = false;
    }
    err = mcpwm_timer_disable(timer_);
    if (err != ESP_OK) {
      logger_.error("Failed to disable MCPWM timer: {}", esp_err_to_name(err));
      ok = false;
    }
    enabled_ = false;
  }
  duty_cycle_ = {0.0f, 0.0f};
  raw_duty_ = {0, 0};
  return ok;
}

bool BdcDriver::is_enabled() const { return enabled_; }

bool BdcDriver::set_duty(float duty_a, float duty_b) {
  if (!initialized_) {
    logger_.error("Cannot set brushed motor duty before initialization");
    return false;
  }
  if (!enabled_) {
    logger_.error("Cannot set brushed motor duty while disabled");
    return false;
  }
  bool ok_a = apply_output(0, duty_a);
  bool ok_b = apply_output(1, duty_b);
  return ok_a && ok_b;
}

bool BdcDriver::set_speed(float speed) {
  speed = std::clamp(speed, -1.0f, 1.0f);
  if (speed > 0.0f) {
    return set_duty(speed, 0.0f);
  }
  if (speed < 0.0f) {
    return set_duty(0.0f, -speed);
  }
  return stop();
}

bool BdcDriver::stop() {
  if (!initialized_) {
    logger_.error("Cannot stop brushed motor driver before initialization");
    return false;
  }
  bool ok_a = apply_output(0, 0.0f);
  bool ok_b = apply_output(1, 0.0f);
  return ok_a && ok_b;
}

std::array<float, 2> BdcDriver::duty_cycle() const { return duty_cycle_; }

std::array<uint32_t, 2> BdcDriver::raw_duty() const { return raw_duty_; }

uint32_t BdcDriver::max_raw_duty() const { return period_ticks_; }

size_t BdcDriver::pwm_frequency_hz() const { return pwm_frequency_hz_; }

size_t BdcDriver::timer_resolution_hz() const { return timer_resolution_hz_; }

int BdcDriver::group_id() const { return group_id_; }

gpio_num_t BdcDriver::gpio_a() const { return gpio_a_; }

gpio_num_t BdcDriver::gpio_b() const { return gpio_b_; }

bool BdcDriver::init(const Config &config) {
  if (gpio_a_ == GPIO_NUM_NC || gpio_b_ == GPIO_NUM_NC) {
    logger_.error("BDC driver requires two valid PWM GPIOs");
    return false;
  }
  if (group_id_ < 0) {
    logger_.error("BDC driver requires a non-negative MCPWM group id");
    return false;
  }
  if (timer_resolution_hz_ == 0 || pwm_frequency_hz_ == 0) {
    logger_.error("BDC driver requires non-zero timer resolution and PWM frequency");
    return false;
  }
  period_ticks_ = timer_resolution_hz_ / pwm_frequency_hz_;
  if (period_ticks_ < 2) {
    logger_.error("BDC driver timer period is too small: {} ticks", period_ticks_);
    return false;
  }
  if (!configure_enable_gpio()) {
    return false;
  }
  if (!configure_timer_and_operator()) {
    return false;
  }
  if (!configure_outputs(config)) {
    return false;
  }
  if (!apply_output(0, 0.0f) || !apply_output(1, 0.0f)) {
    return false;
  }
  initialized_ = true;
  if (!enable()) {
    initialized_ = false;
    return false;
  }
  return true;
}

bool BdcDriver::configure_enable_gpio() {
  if (gpio_enable_ == GPIO_NUM_NC) {
    return true;
  }
  gpio_config_t config{};
  config.pin_bit_mask = 1ULL << gpio_enable_;
  config.mode = GPIO_MODE_OUTPUT;
  esp_err_t err = gpio_config(&config);
  if (err != ESP_OK) {
    logger_.error("Failed to configure BDC enable GPIO {}: {}", static_cast<int>(gpio_enable_),
                  esp_err_to_name(err));
    return false;
  }
  gpio_set_level(gpio_enable_, 0);
  return true;
}

bool BdcDriver::configure_timer_and_operator() {
  mcpwm_timer_config_t timer_config{};
  timer_config.group_id = group_id_;
  timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  timer_config.resolution_hz = timer_resolution_hz_;
  timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  timer_config.period_ticks = period_ticks_;
  esp_err_t err = mcpwm_new_timer(&timer_config, &timer_);
  if (err != ESP_OK) {
    logger_.error("Failed to create MCPWM timer in group {}: {}", group_id_, esp_err_to_name(err));
    return false;
  }

  mcpwm_operator_config_t operator_config{};
  operator_config.group_id = group_id_;
  err = mcpwm_new_operator(&operator_config, &operator_);
  if (err != ESP_OK) {
    logger_.error("Failed to create MCPWM operator in group {}: {}", group_id_,
                  esp_err_to_name(err));
    return false;
  }

  err = mcpwm_operator_connect_timer(operator_, timer_);
  if (err != ESP_OK) {
    logger_.error("Failed to connect MCPWM operator to timer: {}", esp_err_to_name(err));
    return false;
  }
  return true;
}

bool BdcDriver::configure_outputs(const Config &config) {
  mcpwm_comparator_config_t comparator_config{};
  comparator_config.flags.update_cmp_on_tez = true;
  mcpwm_generator_config_t generator_a_config{};
  generator_a_config.gen_gpio_num = gpio_a_;
  generator_a_config.flags.invert_pwm = config.invert_output_a;
  mcpwm_generator_config_t generator_b_config{};
  generator_b_config.gen_gpio_num = gpio_b_;
  generator_b_config.flags.invert_pwm = config.invert_output_b;

  const std::array<mcpwm_generator_config_t, 2> generator_configs{generator_a_config,
                                                                  generator_b_config};
  for (size_t i = 0; i < comparators_.size(); i++) {
    esp_err_t err = mcpwm_new_comparator(operator_, &comparator_config, &comparators_[i]);
    if (err != ESP_OK) {
      logger_.error("Failed to create MCPWM comparator {}: {}", i, esp_err_to_name(err));
      return false;
    }
    err = mcpwm_comparator_set_compare_value(comparators_[i], 0);
    if (err != ESP_OK) {
      logger_.error("Failed to initialize MCPWM comparator {}: {}", i, esp_err_to_name(err));
      return false;
    }
    err = mcpwm_new_generator(operator_, &generator_configs[i], &generators_[i]);
    if (err != ESP_OK) {
      logger_.error("Failed to create MCPWM generator {}: {}", i, esp_err_to_name(err));
      return false;
    }
    if (!configure_generator(generators_[i], comparators_[i])) {
      return false;
    }
  }
  return true;
}

bool BdcDriver::configure_generator(mcpwm_gen_handle_t generator, mcpwm_cmpr_handle_t comparator) {
  esp_err_t err = mcpwm_generator_set_action_on_timer_event(
      generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
                                              MCPWM_GEN_ACTION_HIGH));
  if (err != ESP_OK) {
    logger_.error("Failed to set MCPWM generator empty action: {}", esp_err_to_name(err));
    return false;
  }
  err = mcpwm_generator_set_action_on_timer_event(
      generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_FULL,
                                              MCPWM_GEN_ACTION_LOW));
  if (err != ESP_OK) {
    logger_.error("Failed to set MCPWM generator full action: {}", esp_err_to_name(err));
    return false;
  }
  err = mcpwm_generator_set_action_on_compare_event(
      generator,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW));
  if (err != ESP_OK) {
    logger_.error("Failed to set MCPWM generator compare action: {}", esp_err_to_name(err));
    return false;
  }
  return true;
}

bool BdcDriver::apply_output(size_t index, float duty) {
  duty = std::clamp(duty, 0.0f, 1.0f);
  mcpwm_gen_handle_t generator = generators_[index];
  mcpwm_cmpr_handle_t comparator = comparators_[index];
  if (!generator || !comparator) {
    logger_.error("MCPWM output {} is not initialized", index);
    return false;
  }

  esp_err_t err = ESP_OK;
  uint32_t raw = 0;
  if (duty <= 0.0f) {
    err = mcpwm_generator_set_force_level(generator, 0, true);
  } else if (duty >= 1.0f) {
    raw = period_ticks_;
    err = mcpwm_generator_set_force_level(generator, 1, true);
  } else {
    const uint32_t max_compare = period_ticks_ - static_cast<uint32_t>(1);
    raw = std::clamp(static_cast<uint32_t>(duty * static_cast<float>(period_ticks_) + 0.5f),
                     static_cast<uint32_t>(1), max_compare);
    err = mcpwm_comparator_set_compare_value(comparator, raw);
    if (err == ESP_OK) {
      err = mcpwm_generator_set_force_level(generator, -1, true);
    }
  }
  if (err != ESP_OK) {
    logger_.error("Failed to update MCPWM output {}: {}", index, esp_err_to_name(err));
    return false;
  }
  duty_cycle_[index] = duty;
  raw_duty_[index] = raw;
  return true;
}

void BdcDriver::cleanup() {
  disable();
  for (auto &generator : generators_) {
    if (generator) {
      mcpwm_del_generator(generator);
      generator = nullptr;
    }
  }
  for (auto &comparator : comparators_) {
    if (comparator) {
      mcpwm_del_comparator(comparator);
      comparator = nullptr;
    }
  }
  if (operator_) {
    mcpwm_del_operator(operator_);
    operator_ = nullptr;
  }
  if (timer_) {
    mcpwm_del_timer(timer_);
    timer_ = nullptr;
  }
  initialized_ = false;
}
