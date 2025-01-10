#pragma once

#include <algorithm>
#include <cmath>
#include <mutex>
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
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0) || defined(_DOXYGEN_)
    ledc_sleep_mode_t sleep_mode{
        LEDC_SLEEP_MODE_KEEP_ALIVE}; /**< The LEDC sleep mode you want for this
                                        LED channel. Default is
                                        LEDC_SLEEP_MODE_KEEP_ALIVE which will
                                        keep the LEDC output when the system
                                        enters light sleep. Note that this is
                                        only useful if the LED's clock_config is
                                        set to a clock source which supports
                                        light sleep. */
#endif
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
  explicit Led(const Config &config) noexcept;

  /**
   * @brief Stop the LEDC subsystem and free memory.
   */
  ~Led();

  /**
   * @brief Uninstall the fade service. This should be called if you want to
   *       stop the fade service and free up the ISR.
   * @note This function should only be called if you are sure that no other
   *       Led objects are using the fade service.
   */
  static void uninstall_isr();

  /**
   * @brief Can the LED settings can be changed for the channel? If this
   *        function returns true, then (threaded race conditions aside), the
   *        set_duty() and set_fade_with_time() functions should not block.
   * @param channel The channel to check
   * @return True if the channel settings can be changed, false otherwise
   */
  bool can_change(ledc_channel_t channel);

  /**
   * @brief Get the current duty cycle this channel has.
   * @param channel The channel in question
   * @return The duty percentage [0.0f, 100.0f] if the channel is managed,
   *         std::nullopt otherwise
   */
  std::optional<float> get_duty(ledc_channel_t channel) const;

  /**
   * @brief Set the duty cycle for this channel.
   * @note This function will block until until a current fade process
   *        completes (if there is one).
   * @param channel The channel to set the duty cycle for.
   * @param duty_percent The new duty percentage, [0.0, 100.0].
   */
  void set_duty(ledc_channel_t channel, float duty_percent);

  /**
   * @brief Set the duty cycle for this channel, fading from the current duty
   *        cycle to the new duty cycle over \p fade_time_ms milliseconds.
   * @note This function will block until until a current fade process
   *        completes (if there is one).
   * @param channel The channel to fade.
   * @param duty_percent The new duty percentage to fade to, [0.0, 100.0].
   * @param fade_time_ms The number of milliseconds for which to fade.
   */
  void set_fade_with_time(ledc_channel_t channel, float duty_percent, uint32_t fade_time_ms);

protected:
  static std::mutex fade_service_mutex;
  static bool fade_service_installed;

  /**
   * @brief Get the index of channel in channels_, -1 if not found.
   * @note We implement this instead of using std::find because we cannot use
   *       an iterator efficiently with the semaphore array which we need to
   *       do.
   * @param channel Channel to find.
   * @return -1 if not found, index of channel if found.
   */
  int get_channel_index(ledc_channel_t channel) const;

  /**
   * This callback function will be called when fade operation has ended
   * Use callback only if you are aware it is being called inside an ISR
   * Otherwise, you can use a semaphore to unblock tasks
   */
  static bool cb_ledc_fade_end_event(const ledc_cb_param_t *param, void *user_arg);

  ledc_timer_bit_t duty_resolution_;
  uint32_t max_raw_duty_;
  std::vector<SemaphoreHandle_t> fade_semaphores_;
  std::vector<ChannelConfig> channels_;
};
} // namespace espp
