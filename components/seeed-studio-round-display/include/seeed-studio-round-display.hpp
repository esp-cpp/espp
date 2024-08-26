#pragma once

#include "base_component.hpp"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <hal/spi_types.h>

#include "chsc6x.hpp"
#include "gc9a01.hpp"
#include "i2c.hpp"
#include "interrupt.hpp"
#include "touchpad_input.hpp"

namespace espp {
/// The SsRoundDisplay class provides an interface to the Seeed Studio Round
/// Display development board.
///
/// The class provides access to the following features:
/// - Touchpad
/// - Display
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \note You must call set_pin_config() before calling get() for the first
/// time, in order to provide the appropriate pin configuration for the
/// controller board connected to the display. Some pin configuration structures
/// are provided for convenience.
///
/// \section seeed_studio_round_display_example Example
/// \snippet seeed_studio_round_display_example.cpp seeed studio round display example
class SsRoundDisplay : public espp::BaseComponent {
public:
  using Pixel = lv_color16_t;              ///< Alias for the type of pixel the display uses.
  using DisplayDriver = espp::Gc9a01;      ///< Alias for the display driver.
  using TouchDriver = espp::Chsc6x;        ///< Alias for the touch driver.
  using TouchpadData = espp::TouchpadData; ///< Alias for the touchpad data.

  /// The touch callback function type
  /// \param data The touchpad data
  using touch_callback_t = std::function<void(const TouchpadData &)>;

  /// The pin configuration structure
  struct PinConfig {
    gpio_num_t sda = GPIO_NUM_NC;             ///< I2C data
    gpio_num_t scl = GPIO_NUM_NC;             ///< I2C clock
    gpio_num_t usd_cs = GPIO_NUM_NC;          ///< uSD card chip select
    gpio_num_t lcd_cs = GPIO_NUM_NC;          ///< LCD chip select
    gpio_num_t lcd_dc = GPIO_NUM_NC;          ///< LCD data/command
    gpio_num_t lcd_backlight = GPIO_NUM_NC;   ///< LCD backlight
    gpio_num_t miso = GPIO_NUM_NC;            ///< SPI MISO
    gpio_num_t mosi = GPIO_NUM_NC;            ///< SPI MOSI
    gpio_num_t sck = GPIO_NUM_NC;             ///< SPI SCK
    gpio_num_t touch_interrupt = GPIO_NUM_NC; ///< Touch interrupt

    bool operator==(const PinConfig &rhs) const = default;
    bool operator!=(const PinConfig &rhs) const = default;
  };

  /// @brief The default pin configuration for the Seeed Studio Round Display
  ///        connected to the Xiao ESP32-S3
  static constexpr PinConfig XiaoS3Config = {
      .sda = GPIO_NUM_5,              ///< I2C data. D4 on the Xiao
      .scl = GPIO_NUM_6,              ///< I2C clock. D5 on the Xiao
      .usd_cs = GPIO_NUM_3,           ///< uSD card chip select. D2 on the Xiao
      .lcd_cs = GPIO_NUM_2,           ///< LCD chip select. D1 on the Xiao
      .lcd_dc = GPIO_NUM_4,           ///< LCD data/command. D3 on the Xiao
      .lcd_backlight = GPIO_NUM_43,   ///< LCD backlight. D6/TX on the Xiao
      .miso = GPIO_NUM_8,             ///< SPI MISO. D9 / MISO on the Xiao
      .mosi = GPIO_NUM_9,             ///< SPI MOSI. D10 / MOSI on the Xiao
      .sck = GPIO_NUM_7,              ///< SPI SCK. D8 / SCK on the Xiao
      .touch_interrupt = GPIO_NUM_44, ///< Touch interrupt. D7/RX on the Xiao
  };

  /// @brief The default pin configuration for the Seeed Studio Round Display
  ///        connected to the Qtpy Esp32S3
  static constexpr PinConfig QtpyS3Config = {
      .sda = GPIO_NUM_7,              ///< I2C data. SDA on the Qtpy
      .scl = GPIO_NUM_6,              ///< I2C clock. SCL on the Qtpy
      .usd_cs = GPIO_NUM_9,           ///< uSD card chip select. A2 on the Qtpy
      .lcd_cs = GPIO_NUM_17,          ///< LCD chip select. A1 on the Qtpy
      .lcd_dc = GPIO_NUM_8,           ///< LCD data/command. A3 on the Qtpy
      .lcd_backlight = GPIO_NUM_5,    ///< LCD backlight. TX on the Qtpy
      .miso = GPIO_NUM_37,            ///< SPI MISO. MISO on the Qtpy
      .mosi = GPIO_NUM_35,            ///< SPI MOSI. MOSI on the Qtpy
      .sck = GPIO_NUM_36,             ///< SPI SCK. SCK on the Qtpy
      .touch_interrupt = GPIO_NUM_16, ///< Touch interrupt. RX on the Qtpy
  };

  /// @brief Set the pin configuration for the controller board connected to the
  ///        display
  /// @param pin_config The pin configuration for the controller board
  static void set_pin_config(const PinConfig &pin_config) { pin_config_ = pin_config; }

  /// @brief Access the singleton instance of the SsRoundDisplay class
  /// @return Reference to the singleton instance of the SsRoundDisplay class
  /// @note This method must be called after set_pin_config() has been called
  static SsRoundDisplay &get() {
    static SsRoundDisplay instance;
    return instance;
  }

  SsRoundDisplay(const SsRoundDisplay &) = delete;
  SsRoundDisplay &operator=(const SsRoundDisplay &) = delete;
  SsRoundDisplay(SsRoundDisplay &&) = delete;
  SsRoundDisplay &operator=(SsRoundDisplay &&) = delete;

  /// Get a reference to the internal I2C bus
  /// \return A reference to the internal I2C bus
  espp::I2c &internal_i2c();

  /// Get a reference to the interrupts
  /// \return A reference to the interrupts
  espp::Interrupt &interrupts();

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
  bool initialize_display(size_t pixel_buffer_size,
                          const espp::Task::BaseConfig &task_config = {.name = "Display",
                                                                       .stack_size_bytes = 4096,
                                                                       .priority = 10,
                                                                       .core_id = 0},
                          int update_period_ms = 16);

  /// Get the width of the LCD in pixels
  /// \return The width of the LCD in pixels
  static constexpr size_t lcd_width() { return lcd_width_; }

  /// Get the height of the LCD in pixels
  /// \return The height of the LCD in pixels
  static constexpr size_t lcd_height() { return lcd_height_; }

  /// Get the GPIO pin for the LCD data/command signal
  /// \return The GPIO pin for the LCD data/command signal
  static constexpr auto get_lcd_dc_gpio() { return pin_config_.lcd_dc; }

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
  SsRoundDisplay();
  void touch_interrupt_handler(const espp::Interrupt::Event &event);
  bool update_touch();
  void lcd_wait_lines();

  // common:
  // internal i2c (touchscreen)
  static constexpr auto internal_i2c_port = I2C_NUM_0;
  static constexpr auto internal_i2c_clock_speed = 400 * 1000;

  // LCD
  static constexpr size_t lcd_width_ = 240;
  static constexpr size_t lcd_height_ = 240;
  static constexpr size_t lcd_bytes_per_pixel = 2;
  static constexpr size_t frame_buffer_size = (((lcd_width_)*lcd_bytes_per_pixel) * lcd_height_);
  static constexpr int lcd_clock_speed = 20 * 1000 * 1000;
  static constexpr auto lcd_spi_num = SPI2_HOST;
  static constexpr gpio_num_t lcd_reset_io = GPIO_NUM_NC;
  static constexpr bool backlight_value = true;
  static constexpr bool reset_value = false;
  static constexpr bool invert_colors = true;
  static constexpr auto rotation = espp::DisplayRotation::LANDSCAPE;
  static constexpr bool mirror_x = false;
  static constexpr bool mirror_y = false;
  static constexpr bool mirror_portrait = false;
  static constexpr bool swap_xy = false;
  static constexpr bool swap_color_order = true;

  // touch
  static constexpr bool touch_swap_xy = false;
  static constexpr bool touch_invert_x = false;
  static constexpr bool touch_invert_y = false;
  static constexpr auto touch_interrupt_level = espp::Interrupt::ActiveLevel::LOW;

  static PinConfig pin_config_;
  I2c internal_i2c_;
  espp::Interrupt::PinConfig touch_interrupt_pin_;

  // we'll only add each interrupt pin if the initialize method is called
  espp::Interrupt interrupts_{
      {.interrupts = {},
       .task_config = {.name = "round display interrupts",
                       .stack_size_bytes =
                           CONFIG_SEEED_STUDIO_ROUND_DISPLAY_INTERRUPT_STACK_SIZE}}};

  // touch
  std::shared_ptr<TouchDriver> touch_;
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
  uint8_t *frame_buffer0_{nullptr};
  uint8_t *frame_buffer1_{nullptr};

}; // class SsRoundDisplay
} // namespace espp
