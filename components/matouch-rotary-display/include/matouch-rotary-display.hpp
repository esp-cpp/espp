#pragma once

#include <memory>
#include <string>
#include <vector>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <hal/spi_types.h>

#include "abi_encoder.hpp"
#include "base_component.hpp"
#include "cst816.hpp"
#include "gc9a01.hpp"
#include "i2c.hpp"
#include "interrupt.hpp"
#include "touchpad_input.hpp"

namespace espp {
/// The MatouchRotaryDisplay class provides an interface to the MaTouch Rotary
/// Display 1.28" ESP32-S3 development board (Model MTRO128G).
///
/// The class provides access to the following features:
/// - Touchpad (CST816S)
/// - Display (GC9A01)
/// - ABI Encoder
///
/// For more information see:
/// https://github.com/Makerfabs/MaTouch-ESP32-S3-RotaryIPS-Display1.28-GC9A01
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \section matouch_rotary_display_example Example
/// \snippet matouch_rotary_display_example.cpp matouch-rotary-display example
class MatouchRotaryDisplay : public BaseComponent {
public:
  /// Alias for the pixel type used by the Matouch display
  using Pixel = lv_color16_t;

  /// Alias for the display driver used by the Matouch display
  using DisplayDriver = espp::Gc9a01;

  /// The data structure for the touchpad
  struct TouchpadData {
    uint8_t num_touch_points = 0; ///< The number of touch points
    uint16_t x = 0;               ///< The x coordinate
    uint16_t y = 0;               ///< The y coordinate
    uint8_t btn_state = 0;        ///< The button state (0 = button released, 1 = button pressed)

    /// @brief Compare two TouchpadData objects for equality
    /// @param rhs The right hand side of the comparison
    /// @return true if the two TouchpadData objects are equal, false otherwise
    bool operator==(const TouchpadData &rhs) const = default;
  };

  using Encoder = espp::AbiEncoder<espp::EncoderType::ROTATIONAL>;
  using button_callback_t = espp::Interrupt::event_callback_fn;
  using touch_callback_t = std::function<void(const TouchpadData &)>;

  /// @brief Access the singleton instance of the MatouchRotaryDisplay class
  /// @return Reference to the singleton instance of the MatouchRotaryDisplay class
  static MatouchRotaryDisplay &get() {
    static MatouchRotaryDisplay instance;
    return instance;
  }

  MatouchRotaryDisplay(const MatouchRotaryDisplay &) = delete;
  MatouchRotaryDisplay &operator=(const MatouchRotaryDisplay &) = delete;
  MatouchRotaryDisplay(MatouchRotaryDisplay &&) = delete;
  MatouchRotaryDisplay &operator=(MatouchRotaryDisplay &&) = delete;

  /// Get a reference to the internal I2C bus
  /// \return A reference to the internal I2C bus
  /// \note The internal I2C bus is used for the touchscreen
  I2c &internal_i2c();

  /// Get a reference to the interrupts
  /// \return A reference to the interrupts
  espp::Interrupt &interrupts();

  /////////////////////////////////////////////////////////////////////////////
  // Encoder
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the encoder
  /// \return true if the encoder was successfully initialized, false otherwise
  bool initialize_encoder();

  /// Get the encoder
  /// \return A shared pointer to the encoder
  std::shared_ptr<Encoder> encoder() const;

  /// Get the encoder value
  /// \return The encoder value
  int encoder_value();

  /////////////////////////////////////////////////////////////////////////////
  // Button
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the button
  /// \param callback The callback function to call when the button is pressed
  /// \return true if the button was successfully initialized, false otherwise
  bool initialize_button(const button_callback_t &callback = nullptr);

  /// Get the button state
  /// \return The button state (true = button pressed, false = button released)
  bool button_state() const;

  /////////////////////////////////////////////////////////////////////////////
  // Touchpad
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the touchpad
  /// \param callback The callback function to call when the touchpad is
  ///                 touched. This callback will be called after the touchpad
  ///                 data has been updated and will be passed the updated
  ///                 touchpad data.
  /// \return true if the touchpad was successfully initialized, false otherwise
  /// \note This will also register an interrupt for the touchpad which will
  ///       automatically update the touchpad data when the touchpad is touched
  /// \warning This method should be called after the display has been
  ///          initialized if you want the touchpad to be recognized and used
  ///          with LVGL and its objects.
  bool initialize_touch(const touch_callback_t &callback = nullptr);

  /// Get the touchpad input
  /// \return A shared pointer to the touchpad input
  std::shared_ptr<TouchpadInput> touchpad_input() const;

  /// Get the most recent touchpad data
  /// \return The touchpad data
  TouchpadData touchpad_data() const;

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
  /// \param task_config The task configuration for the display task
  /// \param update_period_ms The update period of the display task
  /// \return true if the display was successfully initialized, false otherwise
  /// \note This will also allocate two full frame buffers in the SPIRAM
  bool initialize_display(size_t pixel_buffer_size, const espp::Task::BaseConfig &task_config = {.name="Display", .stack_size_bytes=4096, .priority=10, .core_id=0}, int update_period_ms = 16);

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
  std::shared_ptr<Display<Pixel>> display() const;

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

  /// Get the frame buffer 0 pointer
  /// \return The frame buffer 0 pointer
  /// \note This memory is designed to be used by the application developer and
  ///       is provided as a convenience. It is not used by the display driver.
  /// \note This is null unless initialize_display() has been called
  uint8_t *frame_buffer0() const;

  /// Get the frame buffer 1 pointer
  /// \return The frame buffer 1 pointer
  /// \note This memory is designed to be used by the application developer and
  ///       is provided as a convenience. It is not used by the display driver.
  /// \note This is null unless initialize_display() has been called
  uint8_t *frame_buffer1() const;

  /// Write data to the LCD
  /// \param data The data to write
  /// \param length The length of the data
  /// \param user_data User data to pass to the spi transaction callback
  /// \note This method is designed to be used by the display driver
  /// \note This method queues the data to be written to the LCD, only blocking
  ///      if there is an ongoing SPI transaction
  void write_lcd(const uint8_t *data, size_t length, uint32_t user_data);

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

protected:
  MatouchRotaryDisplay();
  bool update_cst816();
  void lcd_wait_lines();

  // common:
  // internal i2c (touchscreen)
  static constexpr auto internal_i2c_port = I2C_NUM_0;
  static constexpr auto internal_i2c_clock_speed = 400 * 1000;
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_38;
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_39;

  // encoder
  static constexpr gpio_num_t encoder_a = GPIO_NUM_10;
  static constexpr gpio_num_t encoder_b = GPIO_NUM_9;

  // button (press the screen in)
  static constexpr gpio_num_t button_io = GPIO_NUM_6; // active low

  // LCD
  static constexpr size_t lcd_width_ = 240;
  static constexpr size_t lcd_height_ = 240;
  static constexpr size_t lcd_bytes_per_pixel = 2;
  static constexpr size_t frame_buffer_size = (((lcd_width_)*lcd_bytes_per_pixel) * lcd_height_);
  static constexpr int lcd_clock_speed = 40 * 1000 * 1000;
  static constexpr auto lcd_spi_num = SPI2_HOST;
  static constexpr gpio_num_t lcd_cs_io = GPIO_NUM_15;
  static constexpr gpio_num_t lcd_mosi_io = GPIO_NUM_13;
  static constexpr gpio_num_t lcd_sclk_io = GPIO_NUM_14;
  static constexpr gpio_num_t lcd_reset_io = GPIO_NUM_11; // schematic this is marked as LCD_RST
  static constexpr gpio_num_t lcd_dc_io = GPIO_NUM_21;    // schematic this is marked as LCD_RS
  static constexpr bool backlight_value = true;
  static constexpr bool reset_value = false;
  static constexpr bool invert_colors = true;
  static constexpr bool swap_color_order = true;
  static constexpr auto rotation = espp::DisplayRotation::LANDSCAPE;
  static constexpr bool mirror_x = false;
  static constexpr bool mirror_y = false;
  static constexpr gpio_num_t backlight_io = GPIO_NUM_7;

  // touch
  static constexpr bool touch_swap_xy = false;
  static constexpr bool touch_invert_x = false;
  static constexpr bool touch_invert_y = false;
  static constexpr gpio_num_t touch_reset = GPIO_NUM_16;
  static constexpr gpio_num_t touch_interrupt = GPIO_NUM_40;

  // TODO: allow core id configuration
  I2c internal_i2c_{{.port = internal_i2c_port,
                     .sda_io_num = internal_i2c_sda,
                     .scl_io_num = internal_i2c_scl,
                     .sda_pullup_en = GPIO_PULLUP_ENABLE,
                     .scl_pullup_en = GPIO_PULLUP_ENABLE}};

  // encoder
  std::shared_ptr<Encoder> encoder_{nullptr};

  // Interrupts
  espp::Interrupt::PinConfig button_interrupt_pin_{
      .gpio_num = button_io,
      .callback =
          [this](const auto &event) {
            if (button_callback_) {
              button_callback_(event);
            }
          },
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .interrupt_type = espp::Interrupt::Type::ANY_EDGE};
  espp::Interrupt::PinConfig touch_interrupt_pin_{
      .gpio_num = touch_interrupt,
      .callback =
          [this](const auto &event) {
            if (update_cst816()) {
              if (touch_callback_) {
                touch_callback_(touchpad_data());
              }
            }
          },
      .active_level = espp::Interrupt::ActiveLevel::HIGH,
      .interrupt_type = espp::Interrupt::Type::RISING_EDGE};

  // we'll only add each interrupt pin if the initialize method is called
  espp::Interrupt interrupts_{
      {.interrupts = {},
       .task_config = {.name = "matouch interrupts",
                       .stack_size_bytes = CONFIG_MATOUCH_INTERRUPT_STACK_SIZE}}};

  // button
  std::atomic<bool> button_initialized_{false};
  button_callback_t button_callback_{nullptr};

  // touch
  touch_callback_t touch_callback_{nullptr};
  std::shared_ptr<Cst816> cst816_;
  std::shared_ptr<TouchpadInput> touchpad_input_;
  std::recursive_mutex touchpad_data_mutex_;
  TouchpadData touchpad_data_;

  // display
  std::shared_ptr<Display<Pixel>> display_;
  /// SPI bus for communication with the LCD
  spi_bus_config_t lcd_spi_bus_config_;
  spi_device_interface_config_t lcd_config_;
  spi_device_handle_t lcd_handle_{nullptr};
  static constexpr int spi_queue_size = 6;
  spi_transaction_t trans[spi_queue_size];
  std::atomic<int> num_queued_trans = 0;
  uint8_t *frame_buffer0_{nullptr};
  uint8_t *frame_buffer1_{nullptr};
}; // class MatouchRotaryDisplay
} // namespace espp

// for easy printing of TouchpadData using libfmt
template <>
struct fmt::formatter<espp::MatouchRotaryDisplay::TouchpadData> : fmt::formatter<std::string> {
  template <typename FormatContext>
  auto format(const espp::MatouchRotaryDisplay::TouchpadData &c, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(),
                          "TouchpadData{{num_touch_points={}, x={}, y={}, btn_state={}}}",
                          c.num_touch_points, c.x, c.y, c.btn_state);
  }
};
