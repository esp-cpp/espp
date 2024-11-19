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
  bool initialize_led(float breathing_period = 3.5f);

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

  /////////////////////////////////////////////////////////////////////////////
  // Camera
  /////////////////////////////////////////////////////////////////////////////

  /// @brief Get the camera reset pin
  /// @return The camera reset pin
  static gpio_num_t get_camera_reset_pin() { return camera_reset_pin; }

  /// @brief Get the camera XCLK pin
  /// @return The camera XCLK pin
  static gpio_num_t get_camera_xclk_pin() { return camera_xclk_pin; }

  /// @brief Get the camera SDA pin
  /// @return The camera SDA pin
  static gpio_num_t get_camera_sda_pin() { return camera_sda_pin; }

  /// @brief Get the camera SCL pin
  /// @return The camera SCL pin
  static gpio_num_t get_camera_scl_pin() { return camera_scl_pin; }

  /// @brief Get the camera D0 pin
  /// @return The camera D0 pin
  static gpio_num_t get_camera_d0_pin() { return camera_d0_pin; }

  /// @brief Get the camera D1 pin
  /// @return The camera D1 pin
  static gpio_num_t get_camera_d1_pin() { return camera_d1_pin; }

  /// @brief Get the camera D2 pin
  /// @return The camera D2 pin
  static gpio_num_t get_camera_d2_pin() { return camera_d2_pin; }

  /// @brief Get the camera D3 pin
  /// @return The camera D3 pin
  static gpio_num_t get_camera_d3_pin() { return camera_d3_pin; }

  /// @brief Get the camera D4 pin
  /// @return The camera D4 pin
  static gpio_num_t get_camera_d4_pin() { return camera_d4_pin; }

  /// @brief Get the camera D5 pin
  /// @return The camera D5 pin
  static gpio_num_t get_camera_d5_pin() { return camera_d5_pin; }

  /// @brief Get the camera D6 pin
  /// @return The camera D6 pin
  static gpio_num_t get_camera_d6_pin() { return camera_d6_pin; }

  /// @brief Get the camera D7 pin
  /// @return The camera D7 pin
  static gpio_num_t get_camera_d7_pin() { return camera_d7_pin; }

  /// @brief Get the camera VSYNC pin
  /// @return The camera VSYNC pin
  static gpio_num_t get_camera_vsync_pin() { return camera_vsync_pin; }

  /// @brief Get the camera HREF pin
  /// @return The camera HREF pin
  static gpio_num_t get_camera_href_pin() { return camera_href_pin; }

  /// @brief Get the camera PCLK pin
  /// @return The camera PCLK pin
  static gpio_num_t get_camera_pclk_pin() { return camera_pclk_pin; }

  /// @brief Get the camera XCLK frequency
  /// @return The camera XCLK frequency in Hz
  static int get_camera_xclk_freq_hz() { return camera_xclk_freq; }

protected:
  EspTimerCam();
  float led_breathe();
  bool led_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);

  // internal i2c (touchscreen, audio codec)
  static constexpr auto internal_i2c_port = I2C_NUM_0;
  static constexpr auto internal_i2c_clock_speed = 400 * 1000;
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_12;
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_14;

  // Battery
  static constexpr float BATTERY_VOLTAGE_SCALE =
      1.0f / 661.0f; // measured mV across divider to battery volts
  static constexpr gpio_num_t battery_hold_pin = GPIO_NUM_33; // NOTE: unused

  // Camera
  static constexpr gpio_num_t camera_reset_pin = GPIO_NUM_15;
  static constexpr gpio_num_t camera_xclk_pin = GPIO_NUM_27;
  static constexpr gpio_num_t camera_sda_pin = GPIO_NUM_25;
  static constexpr gpio_num_t camera_scl_pin = GPIO_NUM_23;
  static constexpr gpio_num_t camera_d0_pin = GPIO_NUM_32;
  static constexpr gpio_num_t camera_d1_pin = GPIO_NUM_35;
  static constexpr gpio_num_t camera_d2_pin = GPIO_NUM_34;
  static constexpr gpio_num_t camera_d3_pin = GPIO_NUM_5;
  static constexpr gpio_num_t camera_d4_pin = GPIO_NUM_39;
  static constexpr gpio_num_t camera_d5_pin = GPIO_NUM_18;
  static constexpr gpio_num_t camera_d6_pin = GPIO_NUM_36;
  static constexpr gpio_num_t camera_d7_pin = GPIO_NUM_19;
  static constexpr gpio_num_t camera_vsync_pin = GPIO_NUM_22;
  static constexpr gpio_num_t camera_href_pin = GPIO_NUM_26;
  static constexpr gpio_num_t camera_pclk_pin = GPIO_NUM_21;
  static constexpr int camera_xclk_freq = 10 * 1000 * 1000;

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
