#pragma once

#include <memory>
#include <string>
#include <vector>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <hal/spi_types.h>

#include "base_component.hpp"
#include "cst816.hpp"
#include "i2c.hpp"
#include "interrupt.hpp"
#include "led.hpp"
#include "pcf85063.hpp"
#include "qmi8658.hpp"
#include "st7789.hpp"
#include "touchpad_input.hpp"

namespace espp {
/// The WsS3Touch class provides an interface to the Waveshare ESP32-S3 TouchLCD
/// development board(s).
///
/// The class provides access to the following features:
/// - Touchpad (CST816T)
/// - Display (ST7789V2, 240x280)
/// - Audio (buzzer)
/// - Interrupts
/// - Buttons (boot)
/// - I2C
/// - IMU (Inertial Measurement Unit), 6-axis QMI8658
/// - RTC (Real-Time Clock), PCF85063
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \section ws_s3_touch_example Example
/// \snippet sw_s3_touch_example.cpp ws-s3-touch example
class WsS3Touch : public BaseComponent {
public:
  /// Alias for the button callback function
  using button_callback_t = espp::Interrupt::event_callback_fn;

  /// Alias for the pixel type
  using Pixel = lv_color16_t;

  /// Alias for the display driver
  using DisplayDriver = espp::St7789;

  /// Alias for the touch driver
  using TouchDriver = espp::Cst816;

  /// Alias for the touchpad data
  using TouchpadData = espp::TouchpadData;

  /// Alias the IMU
  using Imu = espp::Qmi8658<qmi8658::Interface::I2C>;

  /// Alias the RTC
  using Rtc = espp::Pcf85063;

  /// Alias for the touch callback when touch events are received
  using touch_callback_t = std::function<void(const TouchpadData &)>;

  /// @brief Access the singleton instance of the WsS3Touch class
  /// @return Reference to the singleton instance of the WsS3Touch class
  static WsS3Touch &get() {
    static WsS3Touch instance;
    return instance;
  }

  WsS3Touch(const WsS3Touch &) = delete;
  WsS3Touch &operator=(const WsS3Touch &) = delete;
  WsS3Touch(WsS3Touch &&) = delete;
  WsS3Touch &operator=(WsS3Touch &&) = delete;

  /// Initialize system power control and enable the device
  /// \return true if the system was successfully initialized, false otherwise
  /// \note This is automatically called by the constructor
  bool enable();

  /// Disable system power
  /// \return true if the system power control was successfully disabled
  bool disable();

  /// Get a reference to the internal I2C bus
  /// \return A reference to the internal I2C bus
  /// \note The internal I2C bus is used for the touchscreen, IMU, and RTC
  I2c &internal_i2c() { return internal_i2c_; }

  /// Get a reference to the interrupts
  /// \return A reference to the interrupts
  espp::Interrupt &interrupts() { return interrupts_; }

  /////////////////////////////////////////////////////////////////////////////
  // Touchpad
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the touchpad
  /// \param callback The touchpad callback
  /// \return true if the touchpad was successfully initialized, false otherwise
  /// \warning This method should be called after the display has been
  ///          initialized if you want the touchpad to be recognized and used
  ///          with LVGL and its objects.
  /// \note This will configure the touchpad interrupt pin which will
  ///       automatically call the touch callback function when the touchpad is
  ///       touched
  bool initialize_touch(const touch_callback_t &callback = nullptr);

  /// Get the touchpad input
  /// \return A shared pointer to the touchpad input
  std::shared_ptr<TouchpadInput> touchpad_input() const { return touchpad_input_; }

  /// Get the most recent touchpad data
  /// \return The touchpad data
  TouchpadData touchpad_data() const { return touchpad_data_; }

  /// Get the most recent touchpad data
  /// \param num_touch_points The number of touch points
  /// \param x The x coordinate
  /// \param y The y coordinate
  /// \param btn_state The button state (0 = button released, 1 = button pressed)
  /// \note This method is a convenience method for integrating with LVGL, the
  ///       data it returns is identical to the data returned by the
  ///       touchpad_data() method
  /// \see touchpad_data()
  void touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y, uint8_t *btn_state);

  /// Convert touchpad data from raw reading to display coordinates
  /// \param data The touchpad data to convert
  /// \return The converted touchpad data
  /// \note Uses the touch_invert_x and touch_invert_y settings to determine
  ///       if the x and y coordinates should be inverted
  TouchpadData touchpad_convert(const TouchpadData &data) const;

  /////////////////////////////////////////////////////////////////////////////
  // Display
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the LCD (low level display driver)
  /// \return true if the LCD was successfully initialized, false otherwise
  bool initialize_lcd();

  /// Initialize the display (lvgl display driver)
  /// \param pixel_buffer_size The size of the pixel buffer
  /// \return true if the display was successfully initialized, false otherwise
  /// \note This will also allocate two full frame buffers in the SPIRAM
  bool initialize_display(size_t pixel_buffer_size);

  /// Get the width of the LCD in pixels
  /// \return The width of the LCD in pixels
  static constexpr size_t lcd_width() { return lcd_width_; }

  /// Get the height of the LCD in pixels
  /// \return The height of the LCD in pixels
  static constexpr size_t lcd_height() { return lcd_height_; }

  /// Get the GPIO pin for the LCD data/command signal
  /// \return The GPIO pin for the LCD data/command signal
  static constexpr auto get_lcd_dc_gpio() { return lcd_dc_io; }

  /// Get a shared pointer to the display
  /// \return A shared pointer to the display
  std::shared_ptr<Display<Pixel>> display() const { return display_; }

  /// Set the brightness of the backlight
  /// \param brightness The brightness of the backlight as a percentage (0 - 100)
  void brightness(float brightness);

  /// Get the brightness of the backlight
  /// \return The brightness of the backlight as a percentage (0 - 100)
  float brightness() const;

  /// Get the VRAM 0 pointer (DMA memory used by LVGL)
  /// \return The VRAM 0 pointer
  /// \note This is the memory used by LVGL for rendering
  /// \note This is null unless initialize_display() has been called
  Pixel *vram0() const;

  /// Get the VRAM 1 pointer (DMA memory used by LVGL)
  /// \return The VRAM 1 pointer
  /// \note This is the memory used by LVGL for rendering
  /// \note This is null unless initialize_display() has been called
  Pixel *vram1() const;

  /// Write command and optional parameters to the LCD
  /// \param command The command to write
  /// \param parameters The command parameters to write
  /// \param user_data User data to pass to the spi transaction callback
  /// \note This method is designed to be used by the display driver
  /// \note This method queues the data to be written to the LCD, only blocking
  ///      if there is an ongoing SPI transaction
  void write_command(uint8_t command, std::span<const uint8_t> parameters, uint32_t user_data);

  /// Write a frame to the LCD
  /// \param x The x coordinate
  /// \param y The y coordinate
  /// \param width The width of the frame, in pixels
  /// \param height The height of the frame, in pixels
  /// \param data The data to write
  /// \note This method queues the data to be written to the LCD, only blocking
  ///      if there is an ongoing SPI transaction
  void write_lcd_frame(const uint16_t x, const uint16_t y, const uint16_t width,
                       const uint16_t height, uint8_t *data);

  /// Write lines to the LCD
  /// \param xs The x start coordinate
  /// \param ys The y start coordinate
  /// \param xe The x end coordinate
  /// \param ye The y end coordinate
  /// \param data The data to write
  /// \param user_data User data to pass to the spi transaction callback
  /// \note This method queues the data to be written to the LCD, only blocking
  ///      if there is an ongoing SPI transaction
  void write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data, uint32_t user_data);

  /////////////////////////////////////////////////////////////////////////////
  // Button
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the boot button (middle button)
  /// \param callback The callback function to call when the button is pressed
  /// \return true if the button was successfully initialized, false otherwise
  bool initialize_boot_button(const button_callback_t &callback = nullptr);

  /// Get the boot button state
  /// \return The button state (true = button pressed, false = button released)
  bool boot_button_state() const;

  /////////////////////////////////////////////////////////////////////////////
  // Audio (buzzer)
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the audio (buzzer)
  /// \return true if the audio was successfully initialized, false otherwise
  bool initialize_buzzer();

  /// Play audio
  /// \param pwm The PWM value to use for the audio output (0.0 - 100.0)
  /// \param frequency_hz The frequency of the audio output in Hz (default is 1000)
  bool buzzer(float pwm, size_t frequency_hz = 1000);

  /////////////////////////////////////////////////////////////////////////////
  // IMU
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the IMU
  /// \param orientation_filter The orientation filter, if provided
  /// \param imu_config The IMU configuration
  /// \return true if the IMU was successfully initialized, false otherwise
  bool initialize_imu(const Imu::filter_fn &orientation_filter = nullptr,
                      const Imu::ImuConfig &imu_config = {
                          .accelerometer_range = Imu::AccelerometerRange::RANGE_8G,
                          .accelerometer_odr = Imu::ODR::ODR_250_HZ,
                          .gyroscope_range = Imu::GyroscopeRange::RANGE_2048_DPS,
                          .gyroscope_odr = Imu::ODR::ODR_250_HZ,
                      });

  /// Get the IMU
  /// \return A shared pointer to the IMU
  std::shared_ptr<Imu> imu() const;

  /////////////////////////////////////////////////////////////////////////////
  // RTC
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the RTC
  /// \return true if the RTC was successfully initialized, false otherwise
  bool initialize_rtc();

  /// Get the RTC
  /// \return A shared pointer to the RTC
  std::shared_ptr<Rtc> rtc() const;

protected:
  WsS3Touch();
  bool update_touch();
  bool audio_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);
  void lcd_wait_lines();

#if CONFIG_WS_S3_TOUCH_VERSION_NEW
  // sound
  static constexpr gpio_num_t buzzer_io = GPIO_NUM_42;
  // rtc
  static constexpr gpio_num_t rtc_interrupt = GPIO_NUM_39;
  // system
  static constexpr gpio_num_t sys_enable_io = GPIO_NUM_41;
  static constexpr gpio_num_t sys_sense_io = GPIO_NUM_40;
#elif CONFIG_WS_S3_TOUCH_VERSION_OLD
  // sound
  static constexpr gpio_num_t buzzer_io = GPIO_NUM_33;
  // rtc
  static constexpr gpio_num_t rtc_interrupt = GPIO_NUM_41;
  // system
  static constexpr gpio_num_t sys_enable_io = GPIO_NUM_35;
  static constexpr gpio_num_t sys_sense_io = GPIO_NUM_36;
#else
#error                                                                                             \
    "Unknown WS-S3-Touch version, please set CONFIG_WS_S3_TOUCH_VERSION_NEW or CONFIG_WS_S3_TOUCH_VERSION_OLD"
#endif

  // common:
  // buttons
  static constexpr gpio_num_t boot_button_io = GPIO_NUM_0; // active low

  // battery (adc)
  static constexpr gpio_num_t bat_adc_io = GPIO_NUM_1;

  // internal i2c (touchscreen, imu, rtc)
  static constexpr auto internal_i2c_port = I2C_NUM_0;
  static constexpr auto internal_i2c_clock_speed = 400 * 1000;
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_11;
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_10;

  // LCD
  static constexpr size_t lcd_width_ = 240;
  static constexpr size_t lcd_height_ = 320;
  static constexpr size_t lcd_bytes_per_pixel = 2;
  static constexpr int lcd_clock_speed = 60 * 1000 * 1000;
  static constexpr auto lcd_spi_num = SPI2_HOST;
  static constexpr gpio_num_t lcd_dc_io = GPIO_NUM_4;
  static constexpr gpio_num_t lcd_cs_io = GPIO_NUM_5;
  static constexpr gpio_num_t lcd_sclk_io = GPIO_NUM_6;
  static constexpr gpio_num_t lcd_mosi_io = GPIO_NUM_7;
  static constexpr gpio_num_t lcd_reset_io = GPIO_NUM_8;
  static constexpr gpio_num_t backlight_io = GPIO_NUM_15;
  static constexpr bool reset_value = false;
  static constexpr bool backlight_value = true;
  static constexpr bool invert_colors = true;
  static constexpr auto rotation = espp::DisplayRotation::LANDSCAPE;
  static constexpr bool mirror_x = false;
  static constexpr bool mirror_y = false;
  static constexpr bool swap_xy = false;
  static constexpr bool swap_color_order = true;

  // sound
  static constexpr size_t buzzer_default_frequency_hz = 5000;
  static constexpr auto buzzer_duty_resolution = LEDC_TIMER_13_BIT;
  static constexpr auto buzzer_ledc_timer = LEDC_TIMER_1;
  static constexpr auto buzzer_ledc_channel = LEDC_CHANNEL_1;

  // touch
  static constexpr bool touch_swap_xy = false;
  static constexpr bool touch_invert_y = false;
  static constexpr bool touch_invert_x = false;
  static constexpr gpio_num_t touch_interrupt = GPIO_NUM_14;
  static constexpr gpio_num_t touch_reset = GPIO_NUM_13;
  static constexpr auto touch_interrupt_level = espp::Interrupt::ActiveLevel::HIGH;
  static constexpr auto touch_interrupt_type = espp::Interrupt::Type::RISING_EDGE;
  static constexpr auto touch_interrupt_pullup_enabled = false;

  // TODO: allow core id configuration
  I2c internal_i2c_{{.port = internal_i2c_port,
                     .sda_io_num = internal_i2c_sda,
                     .scl_io_num = internal_i2c_scl,
                     .sda_pullup_en = GPIO_PULLUP_ENABLE,
                     .scl_pullup_en = GPIO_PULLUP_ENABLE}};

  espp::Interrupt::PinConfig boot_button_interrupt_pin_{
      .gpio_num = boot_button_io,
      .callback =
          [this](const auto &event) {
            if (boot_button_callback_) {
              boot_button_callback_(event);
            }
          },
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
      .pullup_enabled = true};
  // NOTE: the active level, interrupt type, and pullup configuration is set by
  // detect(), since it depends on the box type
  espp::Interrupt::PinConfig touch_interrupt_pin_{
      .gpio_num = touch_interrupt,
      .callback =
          [this](const auto &event) {
            if (update_touch()) {
              if (touch_callback_) {
                touch_callback_(touchpad_data());
              }
            }
          },
      .active_level = touch_interrupt_level,
      .filter_type = espp::Interrupt::FilterType::PIN_GLITCH_FILTER,
  };

  // we'll only add each interrupt pin if the initialize method is called
  espp::Interrupt interrupts_{
      {.interrupts = {},
       .task_config = {.name = "ws-s3-touch interrupts",
                       .stack_size_bytes = CONFIG_WS_S3_TOUCH_INTERRUPT_STACK_SIZE}}};

  // button
  std::atomic<bool> boot_button_initialized_{false};
  button_callback_t boot_button_callback_{nullptr};

  // touch
  std::shared_ptr<TouchDriver> touch_driver_;
  std::shared_ptr<TouchpadInput> touchpad_input_;
  std::recursive_mutex touchpad_data_mutex_;
  TouchpadData touchpad_data_;
  touch_callback_t touch_callback_{nullptr};

  // display
  std::shared_ptr<Display<Pixel>> display_;
  /// SPI bus for communication with the LCD
  spi_bus_config_t lcd_spi_bus_config_;
  spi_device_interface_config_t lcd_config_;
  spi_device_handle_t lcd_handle_{nullptr};
  static constexpr int spi_queue_size = 6;
  spi_transaction_t trans[spi_queue_size];
  std::atomic<int> num_queued_trans = 0;

  // sound
  std::shared_ptr<espp::Led> buzzer_;
  espp::Led::ChannelConfig buzzer_channel_config_{
      .gpio = buzzer_io,
      .channel = buzzer_ledc_channel,
      .timer = buzzer_ledc_timer,
      .output_invert = false,
  };

  // IMU
  std::shared_ptr<Imu> imu_;
  std::shared_ptr<Rtc> rtc_;
}; // class WsS3Touch
} // namespace espp
