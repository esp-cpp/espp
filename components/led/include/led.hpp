#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "base_component.hpp"
#include "run_on_core.hpp"

namespace espp {
/**
 *  Provides a wrapper around the LEDC peripheral in ESP-IDF which allows for
 *  thread-safe control over one or more channels of LEDs using a simpler API.
 *
 * \section led_ex1 Linear LED Example
 * \snippet led_example.cpp linear led example
 * \section led_ex2 Breathing LED Example
 * \snippet led_example.cpp breathing led example
 */
class Led : public BaseComponent {
public:
  /**
   *  Represents one LED channel.
   */
  struct ChannelConfig {
    size_t gpio;            /**< The GPIO pin the LED is connected to. */
    ledc_channel_t channel; /**< The LEDC channel that you want associated with this LED. */
    ledc_timer_t timer;     /**< The LEDC timer that you want associated with this LED channel. */
    float duty{
        0}; /**< The starting duty cycle (%) [0, 100] that you want the LED channel to have. */
    ledc_mode_t speed_mode{
        LEDC_LOW_SPEED_MODE};  /**< The LEDC speed mode you want for this LED channel. */
    bool output_invert{false}; /**< Whether to invert the GPIO output for this LED channel. */
  };

  /**
   *  Configuration Struct for the LEDC subsystem including the different LED
   *  channels that should be associated.
   */
  struct Config {
    int isr_core_id = -1; /**< The core to install the LEDC fade function (interrupt) on. If
                            -1, then the LEDC interrupt is installed on the core that this
                            constructor is called on. If 0 or 1, then the LEDC interrupt is
                            installed on the specified core. */
    ledc_timer_t timer;   /**< The LEDC timer that you want associated with the LEDs. */
    size_t frequency_hz;  /**< The frequency that you want to run the PWM hardawre for the LEDs at.
                             @note this is inversely related to the duty resolution configuration. */
    std::vector<ChannelConfig> channels; /**< The LED channels that you want to control. */
    ledc_timer_bit_t duty_resolution{
        LEDC_TIMER_13_BIT}; /**< The resolution of the duty cycle for these LEDs. @note this is
                               inversely related to the frequency configuration. */
    ledc_clk_cfg_t clock_config{
        LEDC_AUTO_CLK}; /**< The LEDC clock configuration you want for these LED channels. */
    ledc_mode_t speed_mode{
        LEDC_LOW_SPEED_MODE}; /**< The LEDC speed mode you want for these LED channels. */
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Log verbosity for the task.  */
  };

  /**
   * @brief Initialize the LEDC subsystem according to the configuration.
   * @param config The configuration structure for the LEDC subsystem.
   */
  explicit Led(const Config &config) noexcept
      : BaseComponent("Led", config.log_level)
      , duty_resolution_(config.duty_resolution)
      , max_raw_duty_((uint32_t)(std::pow(2, (int)duty_resolution_) - 1))
      , channels_(config.channels) {

    logger_.info("Initializing timer");
    ledc_timer_config_t ledc_timer;
    memset(&ledc_timer, 0, sizeof(ledc_timer));
    ledc_timer.duty_resolution = duty_resolution_;
    ledc_timer.freq_hz = config.frequency_hz;
    ledc_timer.speed_mode = config.speed_mode;
    ledc_timer.timer_num = config.timer;
    ledc_timer.clk_cfg = config.clock_config;
    ledc_timer_config(&ledc_timer);

    logger_.info("Initializing channels");
    for (const auto &conf : channels_) {
      uint32_t actual_duty = std::clamp(conf.duty, 0.0f, 100.0f) * max_raw_duty_ / 100.0f;
      ledc_channel_config_t channel_conf;
      memset(&channel_conf, 0, sizeof(channel_conf));
      channel_conf.channel = conf.channel;
      channel_conf.duty = actual_duty;
      channel_conf.gpio_num = conf.gpio;
      channel_conf.speed_mode = conf.speed_mode;
      channel_conf.hpoint = 0;
      channel_conf.timer_sel = conf.timer;
      channel_conf.flags.output_invert = conf.output_invert;
      ledc_channel_config(&channel_conf);
    }

    logger_.info("Initializing the fade service");
    static bool fade_service_installed = false;
    if (!fade_service_installed) {
      auto install_fn = []() -> esp_err_t { return ledc_fade_func_install(0); };
      auto err = espp::task::run_on_core(install_fn, config.isr_core_id);
      if (err != ESP_OK) {
        logger_.error("install ledc fade service failed {}", esp_err_to_name(err));
      }
      fade_service_installed = err == ESP_OK;
    }

    ledc_cbs_t callbacks = {.fade_cb = &Led::cb_ledc_fade_end_event};

    // we associate each channel with its own semaphore so that they can be
    // faded / controlled independently.
    logger_.info("Creating semaphores");
    fade_semaphores_.resize(channels_.size());
    for (auto &sem : fade_semaphores_) {
      sem = xSemaphoreCreateBinary();
      // go ahead and give to the semaphores so the functions will work
      xSemaphoreGive(sem);
    }
    for (int i = 0; i < channels_.size(); i++) {
      ledc_cb_register(channels_[i].speed_mode, channels_[i].channel, &callbacks,
                       (void *)fade_semaphores_[i]);
    }
  }

  /**
   * @brief Stop the LEDC subsystem and free memory.
   */
  ~Led() {
    // clean up the semaphores
    for (auto &sem : fade_semaphores_) {
      // take the semaphore (so that we don't delete it until no one is
      // blocked on it)
      xSemaphoreTake(sem, portMAX_DELAY);
      // and delete it
      vSemaphoreDelete(sem);
    }
    ledc_fade_func_uninstall();
  }

  /**
   * @brief Can the LED settings can be changed for the channel? If this
   *        function returns true, then (threaded race conditions aside), the
   *        set_duty() and set_fade_with_time() functions should not block.
   * @param channel The channel to check
   * @return True if the channel settings can be changed, false otherwise
   */
  bool can_change(ledc_channel_t channel) {
    int index = get_channel_index(channel);
    if (index == -1) {
      return false;
    }
    auto &sem = fade_semaphores_[index];
    return uxSemaphoreGetCount(sem) == 1;
  }

  /**
   * @brief Get the current duty cycle this channel has.
   * @param channel The channel in question
   * @return The duty percentage [0.0f, 100.0f] if the channel is managed,
   *         std::nullopt otherwise
   */
  std::optional<float> get_duty(ledc_channel_t channel) const {
    int index = get_channel_index(channel);
    if (index == -1) {
      return {};
    }
    const auto &conf = channels_[index];
    auto raw_duty = ledc_get_duty(conf.speed_mode, conf.channel);
    return (float)raw_duty / (float)max_raw_duty_ * 100.0f;
  }

  /**
   * @brief Set the duty cycle for this channel.
   * @note This function will block until until a current fade process
   *        completes (if there is one).
   * @param channel The channel to set the duty cycle for.
   * @param duty_percent The new duty percentage, [0.0, 100.0].
   */
  void set_duty(ledc_channel_t channel, float duty_percent) {
    int index = get_channel_index(channel);
    if (index == -1) {
      return;
    }
    auto conf = channels_[index];
    auto &sem = fade_semaphores_[index];
    // ensure that it's not fading if it is
    xSemaphoreTake(sem, portMAX_DELAY);
    uint32_t actual_duty = std::clamp(duty_percent, 0.0f, 100.0f) * max_raw_duty_ / 100.0f;
    ledc_set_duty(conf.speed_mode, conf.channel, actual_duty);
    ledc_update_duty(conf.speed_mode, conf.channel);
    // make sure others can set this channel now as well
    xSemaphoreGive(sem);
  }

  /**
   * @brief Set the duty cycle for this channel, fading from the current duty
   *        cycle to the new duty cycle over \p fade_time_ms milliseconds.
   * @note This function will block until until a current fade process
   *        completes (if there is one).
   * @param channel The channel to fade.
   * @param duty_percent The new duty percentage to fade to, [0.0, 100.0].
   * @param fade_time_ms The number of milliseconds for which to fade.
   */
  void set_fade_with_time(ledc_channel_t channel, float duty_percent, uint32_t fade_time_ms) {
    int index = get_channel_index(channel);
    if (index == -1) {
      return;
    }
    auto conf = channels_[index];
    auto &sem = fade_semaphores_[index];
    // ensure that it's not fading if it is
    xSemaphoreTake(sem, portMAX_DELAY);
    uint32_t actual_duty = std::clamp(duty_percent, 0.0f, 100.0f) * max_raw_duty_ / 100.0f;
    ledc_set_fade_with_time(conf.speed_mode, conf.channel, actual_duty, fade_time_ms);
    ledc_fade_start(conf.speed_mode, conf.channel, LEDC_FADE_NO_WAIT);
    // NOTE: we don't give the semaphore back here because that is the job of
    // the ISR
  }

protected:
  /**
   * @brief Get the index of channel in channels_, -1 if not found.
   * @note We implement this instead of using std::find because we cannot use
   *       an iterator efficiently with the semaphore array which we need to
   *       do.
   * @param channel Channel to find.
   * @return -1 if not found, index of channel if found.
   */
  int get_channel_index(ledc_channel_t channel) const {
    for (int i = 0; i < channels_.size(); i++) {
      if (channels_[i].channel == channel) {
        return i;
      }
    }
    return -1;
  }

  /**
   * This callback function will be called when fade operation has ended
   * Use callback only if you are aware it is being called inside an ISR
   * Otherwise, you can use a semaphore to unblock tasks
   */
  static bool IRAM_ATTR cb_ledc_fade_end_event(const ledc_cb_param_t *param, void *user_arg) {
    portBASE_TYPE taskAwoken = pdFALSE;

    if (param->event == LEDC_FADE_END_EVT) {
      SemaphoreHandle_t sem = (SemaphoreHandle_t)user_arg;
      xSemaphoreGiveFromISR(sem, &taskAwoken);
    }

    return (taskAwoken == pdTRUE);
  }

  ledc_timer_bit_t duty_resolution_;
  uint32_t max_raw_duty_;
  std::vector<SemaphoreHandle_t> fade_semaphores_;
  std::vector<ChannelConfig> channels_;
};
} // namespace espp
