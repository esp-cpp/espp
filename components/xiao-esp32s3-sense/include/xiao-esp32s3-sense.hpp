#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include <driver/gpio.h>
#include <driver/i2s_pdm.h>

#include "base_component.hpp"
#include "gaussian.hpp"
#include "led.hpp"
#include "task.hpp"

namespace espp {
/// The XiaoEsp32S3Sense class provides an interface to the Seeed Studio XIAO
/// ESP32S3 Sense board.
///
/// The class provides access to the following features:
/// - OV2640 camera pin mapping
/// - PDM microphone pin mapping
/// - User LED control / gaussian breathing helpers
/// - User LED pin
/// - microSD SPI pin mapping
/// - Default ESP-IDF PDM RX microphone configuration helpers
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \section xiao_esp32s3_sense_example Example
/// \snippet xiao_esp32s3_sense_example.cpp xiao esp32s3 sense example
class XiaoEsp32S3Sense : public BaseComponent {
public:
  /// Camera pin mapping for the Sense expansion board.
  struct CameraPins {
    gpio_num_t pwdn{GPIO_NUM_NC};     ///< Camera power down pin.
    gpio_num_t reset{GPIO_NUM_NC};    ///< Camera reset pin.
    gpio_num_t xclk{GPIO_NUM_NC};     ///< Camera external clock pin.
    gpio_num_t sccb_sda{GPIO_NUM_NC}; ///< Camera SCCB SDA pin.
    gpio_num_t sccb_scl{GPIO_NUM_NC}; ///< Camera SCCB SCL pin.
    gpio_num_t d7{GPIO_NUM_NC};       ///< Camera D7 / Y9 pin.
    gpio_num_t d6{GPIO_NUM_NC};       ///< Camera D6 / Y8 pin.
    gpio_num_t d5{GPIO_NUM_NC};       ///< Camera D5 / Y7 pin.
    gpio_num_t d4{GPIO_NUM_NC};       ///< Camera D4 / Y6 pin.
    gpio_num_t d3{GPIO_NUM_NC};       ///< Camera D3 / Y5 pin.
    gpio_num_t d2{GPIO_NUM_NC};       ///< Camera D2 / Y4 pin.
    gpio_num_t d1{GPIO_NUM_NC};       ///< Camera D1 / Y3 pin.
    gpio_num_t d0{GPIO_NUM_NC};       ///< Camera D0 / Y2 pin.
    gpio_num_t vsync{GPIO_NUM_NC};    ///< Camera VSYNC pin.
    gpio_num_t href{GPIO_NUM_NC};     ///< Camera HREF pin.
    gpio_num_t pclk{GPIO_NUM_NC};     ///< Camera pixel clock pin.
  };

  /// PDM microphone pin mapping for the Sense expansion board.
  struct MicrophonePins {
    gpio_num_t clk{GPIO_NUM_NC};  ///< PDM clock output pin.
    gpio_num_t data{GPIO_NUM_NC}; ///< PDM data input pin.
  };

  /// SPI microSD pin mapping for the Sense expansion board.
  struct SdCardPins {
    gpio_num_t clk{GPIO_NUM_NC};  ///< SPI clock pin.
    gpio_num_t mosi{GPIO_NUM_NC}; ///< SPI MOSI pin.
    gpio_num_t miso{GPIO_NUM_NC}; ///< SPI MISO pin.
    gpio_num_t cs{GPIO_NUM_NC};   ///< SPI chip-select pin.
  };

  /// @brief Access the singleton instance of the XiaoEsp32S3Sense class.
  /// @return Reference to the singleton instance of the XiaoEsp32S3Sense class.
  static XiaoEsp32S3Sense &get() {
    static XiaoEsp32S3Sense instance;
    return instance;
  }

  XiaoEsp32S3Sense(const XiaoEsp32S3Sense &) = delete;
  XiaoEsp32S3Sense &operator=(const XiaoEsp32S3Sense &) = delete;
  XiaoEsp32S3Sense(XiaoEsp32S3Sense &&) = delete;
  XiaoEsp32S3Sense &operator=(XiaoEsp32S3Sense &&) = delete;

  /// Get the documented camera pin mapping.
  /// \return The camera pin mapping for the XIAO ESP32S3 Sense.
  CameraPins camera_pins() const;

  /// Get the documented PDM microphone pin mapping.
  /// \return The microphone pin mapping for the XIAO ESP32S3 Sense.
  MicrophonePins microphone_pins() const;

  /// Get the documented SPI microSD pin mapping.
  /// \return The microSD pin mapping for the XIAO ESP32S3 Sense.
  SdCardPins sd_card_pins() const;

  /////////////////////////////////////////////////////////////////////////////
  // LED
  /////////////////////////////////////////////////////////////////////////////

  /// @brief Initialize the user LED.
  /// @param breathing_period The period of the LED breathing effect in seconds.
  /// @return True if the LED was successfully initialized, false otherwise.
  bool initialize_led(float breathing_period = 3.5f);

  /// @brief Start the user-LED breathing effect.
  /// @details Logs an error and does nothing if the LED helper has not been
  ///          initialized yet.
  void start_led_breathing();

  /// @brief Stop the user-LED breathing effect.
  /// @details Logs an error and does nothing if the LED helper has not been
  ///          initialized yet.
  void stop_led_breathing();

  /// @brief Set the user-LED brightness.
  /// @param brightness The brightness of the LED, in the range [0, 1].
  /// @return True if the LED brightness was successfully set, false otherwise.
  /// @note The LED brightness can only be set while the breathing task is stopped.
  bool set_led_brightness(float brightness);

  /// @brief Get the user-LED brightness.
  /// @return The brightness of the LED, in the range [0, 1].
  float get_led_brightness();

  /// @brief Set the user-LED breathing period.
  /// @param period The period of the LED breathing effect in seconds.
  /// @return True if the LED breathing period was successfully set, false otherwise.
  bool set_led_breathing_period(float period);

  /// @brief Get the user-LED breathing period.
  /// @return The period of the LED breathing effect in seconds.
  float get_led_breathing_period();

  /// @brief Get the user LED.
  /// @return A shared pointer to the LED.
  std::shared_ptr<espp::Led> led();

  /// @brief Get the Gaussian waveform used for the LED breathing effect.
  /// @return A reference to the Gaussian waveform.
  espp::Gaussian &gaussian();

  /// Get a default PDM RX clock configuration for the onboard microphone.
  /// \param sample_rate_hz The desired microphone sample rate in Hz.
  /// \return The default ESP-IDF PDM RX clock configuration.
  i2s_pdm_rx_clk_config_t
  microphone_clock_config(uint32_t sample_rate_hz = microphone_default_sample_rate_hz()) const;

  /// Get a default PDM RX slot configuration for the onboard microphone.
  /// \param slot_mode The microphone slot mode.
  /// \return The default ESP-IDF PDM RX slot configuration.
  i2s_pdm_rx_slot_config_t
  microphone_slot_config(i2s_slot_mode_t slot_mode = I2S_SLOT_MODE_MONO) const;

  /// Get a default PDM RX GPIO configuration for the onboard microphone.
  /// \param clock_inverted Whether to invert the generated PDM clock.
  /// \return The default ESP-IDF PDM RX GPIO configuration.
  i2s_pdm_rx_gpio_config_t microphone_gpio_config(bool clock_inverted = false) const;

  /// Get a complete default PDM RX configuration for the onboard microphone.
  /// \param sample_rate_hz The desired microphone sample rate in Hz.
  /// \param slot_mode The microphone slot mode.
  /// \param clock_inverted Whether to invert the generated PDM clock.
  /// \return The default ESP-IDF PDM RX configuration.
  i2s_pdm_rx_config_t
  microphone_config(uint32_t sample_rate_hz = microphone_default_sample_rate_hz(),
                    i2s_slot_mode_t slot_mode = I2S_SLOT_MODE_MONO,
                    bool clock_inverted = false) const;

  /// Get the recommended camera XCLK frequency in Hz.
  /// \return The camera XCLK frequency in Hz.
  static constexpr int camera_xclk_freq_hz() { return camera_xclk_freq_hz_; }

  /// Get the default microphone sample rate in Hz.
  /// \return The default microphone sample rate in Hz.
  static constexpr uint32_t microphone_default_sample_rate_hz() {
    return microphone_default_sample_rate_hz_;
  }

  /// Get the user LED pin on the XIAO ESP32S3 module.
  /// \return The user LED GPIO.
  static constexpr gpio_num_t user_led_pin() { return user_led_pin_; }

  /// Get the microSD SPI clock pin on the Sense expansion board.
  /// \return The microSD SPI clock GPIO.
  static constexpr gpio_num_t sd_card_clk_pin() { return sd_card_sck_pin_; }

  /// Get the microSD SPI MOSI pin on the Sense expansion board.
  /// \return The microSD SPI MOSI GPIO.
  static constexpr gpio_num_t sd_card_mosi_pin() { return sd_card_mosi_pin_; }

  /// Get the microSD SPI MISO pin on the Sense expansion board.
  /// \return The microSD SPI MISO GPIO.
  static constexpr gpio_num_t sd_card_miso_pin() { return sd_card_miso_pin_; }

  /// Get the default microSD card CS pin on the Sense expansion board.
  /// \return The microSD card chip select pin.
  static constexpr gpio_num_t sd_card_cs_pin() { return sd_card_cs_pin_; }

  /// Get the camera power down pin.
  static constexpr gpio_num_t camera_pwdn_pin() { return camera_pwdn_pin_; }

  /// Get the camera reset pin.
  static constexpr gpio_num_t camera_reset_pin() { return camera_reset_pin_; }

  /// Get the camera XCLK pin.
  static constexpr gpio_num_t camera_xclk_pin() { return camera_xclk_pin_; }

  /// Get the camera SCCB SDA pin.
  static constexpr gpio_num_t camera_sccb_sda_pin() { return camera_sccb_sda_pin_; }

  /// Get the camera SCCB SCL pin.
  static constexpr gpio_num_t camera_sccb_scl_pin() { return camera_sccb_scl_pin_; }

  /// Get the camera D7 / Y9 pin.
  static constexpr gpio_num_t camera_d7_pin() { return camera_d7_pin_; }

  /// Get the camera D6 / Y8 pin.
  static constexpr gpio_num_t camera_d6_pin() { return camera_d6_pin_; }

  /// Get the camera D5 / Y7 pin.
  static constexpr gpio_num_t camera_d5_pin() { return camera_d5_pin_; }

  /// Get the camera D4 / Y6 pin.
  static constexpr gpio_num_t camera_d4_pin() { return camera_d4_pin_; }

  /// Get the camera D3 / Y5 pin.
  static constexpr gpio_num_t camera_d3_pin() { return camera_d3_pin_; }

  /// Get the camera D2 / Y4 pin.
  static constexpr gpio_num_t camera_d2_pin() { return camera_d2_pin_; }

  /// Get the camera D1 / Y3 pin.
  static constexpr gpio_num_t camera_d1_pin() { return camera_d1_pin_; }

  /// Get the camera D0 / Y2 pin.
  static constexpr gpio_num_t camera_d0_pin() { return camera_d0_pin_; }

  /// Get the camera VSYNC pin.
  static constexpr gpio_num_t camera_vsync_pin() { return camera_vsync_pin_; }

  /// Get the camera HREF pin.
  static constexpr gpio_num_t camera_href_pin() { return camera_href_pin_; }

  /// Get the camera PCLK pin.
  static constexpr gpio_num_t camera_pclk_pin() { return camera_pclk_pin_; }

  /// Get the microphone PDM clock pin.
  static constexpr gpio_num_t microphone_clk_pin() { return microphone_clk_pin_; }

  /// Get the microphone PDM data pin.
  static constexpr gpio_num_t microphone_data_pin() { return microphone_data_pin_; }

protected:
  XiaoEsp32S3Sense();
  float led_breathe();
  bool led_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);

  static constexpr gpio_num_t camera_pwdn_pin_ = GPIO_NUM_NC;
  static constexpr gpio_num_t camera_reset_pin_ = GPIO_NUM_NC;
  static constexpr gpio_num_t camera_xclk_pin_ = GPIO_NUM_10;
  static constexpr gpio_num_t camera_sccb_sda_pin_ = GPIO_NUM_40;
  static constexpr gpio_num_t camera_sccb_scl_pin_ = GPIO_NUM_39;
  static constexpr gpio_num_t camera_d7_pin_ = GPIO_NUM_48;
  static constexpr gpio_num_t camera_d6_pin_ = GPIO_NUM_11;
  static constexpr gpio_num_t camera_d5_pin_ = GPIO_NUM_12;
  static constexpr gpio_num_t camera_d4_pin_ = GPIO_NUM_14;
  static constexpr gpio_num_t camera_d3_pin_ = GPIO_NUM_16;
  static constexpr gpio_num_t camera_d2_pin_ = GPIO_NUM_18;
  static constexpr gpio_num_t camera_d1_pin_ = GPIO_NUM_17;
  static constexpr gpio_num_t camera_d0_pin_ = GPIO_NUM_15;
  static constexpr gpio_num_t camera_vsync_pin_ = GPIO_NUM_38;
  static constexpr gpio_num_t camera_href_pin_ = GPIO_NUM_47;
  static constexpr gpio_num_t camera_pclk_pin_ = GPIO_NUM_13;
  static constexpr int camera_xclk_freq_hz_ = 20 * 1000 * 1000;

  static constexpr gpio_num_t user_led_pin_ = GPIO_NUM_21;

  static constexpr gpio_num_t microphone_clk_pin_ = GPIO_NUM_42;
  static constexpr gpio_num_t microphone_data_pin_ = GPIO_NUM_41;
  static constexpr uint32_t microphone_default_sample_rate_hz_ = 16 * 1000;

  static constexpr gpio_num_t sd_card_mosi_pin_ = GPIO_NUM_9;
  static constexpr gpio_num_t sd_card_miso_pin_ = GPIO_NUM_8;
  static constexpr gpio_num_t sd_card_sck_pin_ = GPIO_NUM_7;
  static constexpr gpio_num_t sd_card_cs_pin_ = GPIO_NUM_3;

  std::vector<espp::Led::ChannelConfig> led_channels_{{
      .gpio = static_cast<size_t>(user_led_pin_),
      .channel = LEDC_CHANNEL_1,
      .timer = LEDC_TIMER_1,
      .output_invert = true,
  }};
  std::shared_ptr<espp::Led> led_;
  std::unique_ptr<espp::Task> led_task_;
  std::atomic<float> breathing_period_{3.5f};
  std::chrono::high_resolution_clock::time_point breathing_start_{};
  espp::Gaussian gaussian_{{.gamma = 0.1f, .alpha = 1.0f, .beta = 0.5f}};
}; // class XiaoEsp32S3Sense
} // namespace espp
