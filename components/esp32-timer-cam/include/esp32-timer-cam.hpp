#pragma once

#include <memory>
#include <string>
#include <vector>

#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <driver/spi_master.h>
#include <hal/spi_types.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>

#include "base_component.hpp"
#include "bm8563.hpp"
#include "gaussian.hpp"
#include "i2c.hpp"
#include "interrupt.hpp"
#include "led.hpp"
#include "oneshot_adc.hpp"

namespace espp {
/// The EspTimerCam class provides an interface to the ESP32-S3-BOX and
/// ESP32-S3-BOX-3 development boards.
///
/// The class provides access to the following features:
/// - I2C
/// - LED
/// - Battery Voltage Measurement
/// - RTC
/// - Camera pin definition
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \section esp_timer_cam_example Example
/// \snippet esp_timer_cam_example.cpp esp timer cam example
class EspTimerCam : public BaseComponent {
public:
  using Rtc = espp::Bm8563;

  /// @brief Access the singleton instance of the EspTimerCam class
  /// @return Reference to the singleton instance of the EspTimerCam class
  static EspTimerCam &get() {
    static EspTimerCam instance;
    return instance;
  }

  EspTimerCam(const EspTimerCam &) = delete;
  EspTimerCam &operator=(const EspTimerCam &) = delete;
  EspTimerCam(EspTimerCam &&) = delete;
  EspTimerCam &operator=(EspTimerCam &&) = delete;

  /// Get a reference to the internal I2C bus
  /// \return A reference to the internal I2C bus
  /// \note The internal I2C bus is used for the touchscreen and audio codec
  I2c &internal_i2c();

  /// Get a reference to the interrupts
  /// \return A reference to the interrupts
  espp::Interrupt &interrupts();

  /////////////////////////////////////////////////////////////////////////////
  // LED
  /////////////////////////////////////////////////////////////////////////////

  /// @brief Initialize the LED
  /// @param breathing_period The period of the LED breathing effect
  /// @return True if the LED was successfully initialized, false otherwise
  bool initialize_led(float breathing_period=3.5f);

  /// @brief Start the LED breathing effect
  void start_led_breathing();

  /// @brief Stop the LED breathing effect
  void stop_led_breathing();

  /// @brief Set the LED brightness
  /// @param brightness The brightness of the LED, in the range [0, 1]
  /// @return True if the LED brightness was successfully set, false otherwise
  /// @note The LED brightness can only be set if the LED is not currently
  ///       breathing
  bool set_led_brightness(float brightness);

  /// @brief Get the LED brightness
  /// @return The brightness of the LED, in the range [0, 1]
  float get_led_brightness();

  /// @brief Set the LED breathing period
  /// @param period The period of the LED breathing effect in seconds
  /// @return True if the LED breathing period was successfully set, false
  ///         otherwise
  bool set_led_breathing_period(float period);

  /// @brief Get the LED breathing period
  /// @return The period of the LED breathing effect in seconds
  float get_led_breathing_period();

  /// @brief Get the LED
  /// @return A shared pointer to the LED
  std::shared_ptr<espp::Led> led();

  /// @brief Get the Gaussian waveform used for the LED breathing effect
  /// @return A reference to the Gaussian waveform used for the LED breathing
  ///         effect
  espp::Gaussian &gaussian();

  /////////////////////////////////////////////////////////////////////////////
  // Battery
  /////////////////////////////////////////////////////////////////////////////

  /// @brief Get the battery voltage
  /// @return The battery voltage in volts
  float get_battery_voltage();

  /////////////////////////////////////////////////////////////////////////////
  // RTC
  /////////////////////////////////////////////////////////////////////////////

  /// @brief Initialize the RTC
  /// @return True if the RTC was successfully initialized, false otherwise
  bool initialize_rtc();

  /// @brief Get the RTC
  /// @return A shared pointer to the RTC
  std::shared_ptr<Rtc> rtc();

protected:
  EspTimerCam();
  float led_breathe();
  bool led_task_callback(std::mutex &m, std::condition_variable &cv);

  // internal i2c (touchscreen, audio codec)
  static constexpr auto internal_i2c_port = I2C_NUM_0;
  static constexpr auto internal_i2c_clock_speed = 400 * 1000;
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_12;
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_14;

  // Camera


  // TODO: allow core id configuration
  I2c internal_i2c_{{.port = internal_i2c_port,
                     .sda_io_num = internal_i2c_sda,
                     .scl_io_num = internal_i2c_scl,
                     .sda_pullup_en = GPIO_PULLUP_ENABLE,
                     .scl_pullup_en = GPIO_PULLUP_ENABLE}};

  // we'll only add each interrupt pin if the initialize method is called
  espp::Interrupt interrupts_{
      {.interrupts = {},
       .task_config = {.name = "esp timer cam interrupts",
                       .stack_size_bytes = CONFIG_ESP_TIMER_CAM_INTERRUPT_STACK_SIZE}}};

  // LED
  std::vector<espp::Led::ChannelConfig> led_channels_{{
      .gpio = 2,
      .channel = LEDC_CHANNEL_5,
      .timer = LEDC_TIMER_2,
  }};
  std::shared_ptr<espp::Led> led_;
  std::unique_ptr<espp::Task> led_task_;
  std::atomic<float> breathing_period_{3.5f};
  std::chrono::high_resolution_clock::time_point breathing_start_{};
  espp::Gaussian gaussian_{{.gamma = 0.1f, .alpha = 1.0f, .beta = 0.5f}};

  // RTC
  std::shared_ptr<Rtc> rtc_;

  // Battery ADC
  espp::AdcConfig battery_channel_{.unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_2, // GPIO 38
      .attenuation = ADC_ATTEN_DB_12};

  // NOTE: for some reason, I cannot use Continuous ADC in combination with
  // esp32-camera...
  espp::OneshotAdc adc_{{.unit = battery_channel_.unit,
                           .channels = {battery_channel_},
                            .log_level = espp::Logger::Verbosity::WARN}};

}; // class EspTimerCam
} // namespace espp
