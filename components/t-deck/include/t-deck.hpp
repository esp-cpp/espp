#pragma once

#include <memory>
#include <string>
#include <vector>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <hal/spi_types.h>

#include "base_component.hpp"
#include "gt911.hpp"
#include "i2c.hpp"
#include "interrupt.hpp"
#include "pointer_input.hpp"
#include "st7789.hpp"
#include "t_keyboard.hpp"
#include "touchpad_input.hpp"

namespace espp {
/// The TDeck class provides an interface to the LilyGo T-Deck ESP32-S3
/// development board.
///
/// The class provides access to the following features:
/// - Touchpad
/// - Display
/// - Keyboard
///
/// For more information, see
/// https://github.com/Xinyuan-LilyGO/T-Deck/tree/master and
/// https://github.com/Xinyuan-LilyGO/T-Deck/blob/master/examples/UnitTest/utilities.h
///
/// \note They keyboard has a backlight, which you can control with the shortcut
///       `alt + b`. The keyboard backlight is off by default.
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \section t_deck_example Example
/// \snippet t_deck_example.cpp t-deck example
class TDeck : public BaseComponent {
public:
  /// Alias for the pixel type used by the TDeck display
  using Pixel = lv_color16_t;

  using keypress_callback_t = TKeyboard::key_cb_fn;
  using touch_callback_t = std::function<void(const espp::TouchpadData &)>;
  using trackball_callback_t = std::function<void(const espp::PointerData &)>;

  /// @brief Access the singleton instance of the TDeck class
  /// @return Reference to the singleton instance of the TDeck class
  static TDeck &get() {
    static TDeck instance;
    return instance;
  }

  TDeck(const TDeck &) = delete;
  TDeck &operator=(const TDeck &) = delete;
  TDeck(TDeck &&) = delete;
  TDeck &operator=(TDeck &&) = delete;

  /// Get a reference to the internal I2C bus
  /// \return A reference to the internal I2C bus
  /// \note The internal I2C bus is used for the touchscreen
  I2c &internal_i2c();

  /// Get a reference to the interrupts
  /// \return A reference to the interrupts
  espp::Interrupt &interrupts();

  /// Get the GPIO pin for the peripheral power
  /// \return The GPIO pin for the peripheral power
  /// \note This pin is used to enable/disable power to the peripherals, such as
  ///       the keyboard, screen, etc.
  static constexpr auto peripheral_power_pin() { return peripheral_power_pin_; }

  /// Enable or disable power to the peripherals
  /// \param on Whether to enable or disable power to the peripherals
  void peripheral_power(bool on);

  /// Get the state of the peripheral power
  /// \return true if power is enabled, false otherwise
  bool peripheral_power() const;

  /////////////////////////////////////////////////////////////////////////////
  // Keyboard
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the keyboard
  /// \param start_task Whether to start the keyboard task
  /// \param key_cb The key callback function, called when a key is pressed if
  ///         not null and the keyboard task is started
  /// \param poll_interval The interval at which to poll the keyboard
  /// \return true if the keyboard was successfully initialized, false otherwise
  /// \note The Keyboard has an interrupt pin connected from it (the esp32c3) to
  ///       the main esp32s3. However, the default firmware on the keyboard
  ///       (esp32c3) does not use this interrupt pin. Instead, the main esp32s3
  ///       must poll the keyboard to get key presses. This is done by the
  ///       keyboard task. If you update the firmware on the keyboard to use the
  ///       interrupt pin, you can set start_task to false and wire up the
  ///       interrupt to the keyboard()->read_key() method.
  /// \see TKeyboard
  /// \see keyboard()
  bool initialize_keyboard(bool start_task = true, const keypress_callback_t &key_cb = nullptr,
                           std::chrono::milliseconds poll_interval = std::chrono::milliseconds(10));

  /// Get the keyboard
  /// \return A shared pointer to the keyboard
  /// \note The keyboard is only available if it was successfully initialized
  std::shared_ptr<TKeyboard> keyboard() const;

  /// Get the GPIO pin for the keyboard interrupt
  /// \return The GPIO pin for the keyboard interrupt
  /// \note This pin is used to detect when a key is pressed on the keyboard
  ///       and is connected to the main esp32s3, however the default firmware
  ///       on the keyboard does not use this pin
  static constexpr auto keyboard_interrupt() { return keyboard_interrupt_io; }

  /////////////////////////////////////////////////////////////////////////////
  // Trackball
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the trackball
  /// \param trackball_cb The trackball callback function, called when the
  ///        trackball is moved if not null
  /// \param sensitivity The sensitivity of the trackball. The higher the
  ///        sensitivity, the faster the trackball will move
  /// \return true if the trackball was successfully initialized, false
  ///         otherwise
  /// \see trackball()
  /// \see trackball_data()
  /// \see trackball_read()
  /// \see set_trackball_sensitivity()
  bool initialize_trackball(const trackball_callback_t &trackball_cb = nullptr,
                            int sensitivity = 10);

  /// Get the trackball
  /// \return A shared pointer to the trackball
  /// \note The trackball is only available if it was successfully initialized
  /// \see initialize_trackball()
  /// \see trackball_data()
  /// \see trackball_read()
  /// \note This is the same as the pointer_input() method
  std::shared_ptr<PointerInput> trackball() const;

  /// Set the trackball sensitivity
  /// \param sensitivity The sensitivity of the trackball. The higher the
  ///       sensitivity, the faster the trackball will move
  /// \note The sensitivity can be negative, which will invert the direction of
  ///       the trackball
  void set_trackball_sensitivity(int sensitivity);

  /// Get the pointer input for the trackball
  /// \return A shared pointer to the pointer input for the trackball
  std::shared_ptr<PointerInput> pointer_input() const;

  /// Get the GPIO pin for the trackball up button
  /// \return The GPIO pin for the trackball up button
  static constexpr auto trackball_up_gpio() { return trackball_up; }

  /// Get the GPIO pin for the trackball down button
  /// \return The GPIO pin for the trackball down button
  static constexpr auto trackball_down_gpio() { return trackball_down; }

  /// Get the GPIO pin for the trackball left button
  /// \return The GPIO pin for the trackball left button
  static constexpr auto trackball_left_gpio() { return trackball_left; }

  /// Get the GPIO pin for the trackball right button
  /// \return The GPIO pin for the trackball right button
  static constexpr auto trackball_right_gpio() { return trackball_right; }

  /// Get the GPIO pin for the trackball button
  /// \return The GPIO pin for the trackball button
  static constexpr auto trackball_btn_gpio() { return trackball_btn; }

  /// Get the most recent trackball data
  /// \return The trackball data
  espp::PointerData trackball_data() const;

  /// Get the most recent trackball data
  /// \param x The x coordinate
  /// \param y The y coordinate
  /// \param left_pressed Whether the left button is pressed
  /// \param right_pressed Whether the right button is pressed
  /// \note This method is a convenience method for integrating with LVGL, the
  ///      data it returns is identical to the data returned by the
  ///      trackball_data() method
  void trackball_read(int &x, int &y, bool &left_pressed, bool &right_pressed);

  /////////////////////////////////////////////////////////////////////////////
  // Touchpad
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the touchpad
  /// \param touch_cb The touch callback function, called when the touchpad is
  ///        touched if not null
  /// \return true if the touchpad was successfully initialized, false otherwise
  /// \warning This method should be called after the display has been
  ///          initialized if you want the touchpad to be recognized and used
  ///          with LVGL and its objects.
  /// \note This will configure the touchpad interrupt pin which will
  ///       automatically call the touch callback function when the touchpad is
  ///       touched
  bool initialize_touch(const touch_callback_t &touch_cb = nullptr);

  /// Get the touchpad input
  /// \return A shared pointer to the touchpad input
  std::shared_ptr<TouchpadInput> touchpad_input() const;

  /// Get the most recent touchpad data
  /// \return The touchpad data
  espp::TouchpadData touchpad_data() const;

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
  espp::TouchpadData touchpad_convert(const espp::TouchpadData &data) const;

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
  TDeck();
  bool update_gt911();
  void lcd_wait_lines();
  void on_trackball_interrupt(const espp::Interrupt::Event &event);

  // common:
  // internal i2c (touchscreen, keyboard)
  static constexpr auto internal_i2c_port = I2C_NUM_0;
  static constexpr auto internal_i2c_clock_speed = 400 * 1000;
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_18;
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_8;

  // peripherals
  static constexpr gpio_num_t peripheral_power_pin_ = GPIO_NUM_10;

  // keyboard
  static constexpr gpio_num_t keyboard_interrupt_io = GPIO_NUM_46; // NOTE: unused by default

  // LCD
  static constexpr size_t lcd_width_ = 320;
  static constexpr size_t lcd_height_ = 240;
  static constexpr size_t lcd_bytes_per_pixel = 2;
  static constexpr size_t frame_buffer_size = (((lcd_width_)*lcd_bytes_per_pixel) * lcd_height_);
  static constexpr int lcd_clock_speed = 40 * 1000 * 1000;
  static constexpr auto lcd_spi_num = SPI2_HOST;
  static constexpr gpio_num_t lcd_cs_io = GPIO_NUM_12;
  static constexpr gpio_num_t lcd_mosi_io = GPIO_NUM_41;
  static constexpr gpio_num_t lcd_sclk_io = GPIO_NUM_40;
  static constexpr gpio_num_t lcd_reset_io = GPIO_NUM_NC;
  static constexpr gpio_num_t lcd_dc_io = GPIO_NUM_11;
  static constexpr bool backlight_value = true;
  static constexpr bool reset_value = false;
  static constexpr bool invert_colors = false;
  static constexpr auto rotation = espp::DisplayRotation::LANDSCAPE;
  static constexpr bool mirror_x = false;
  static constexpr bool mirror_y = false;
  static constexpr bool mirror_portrait = true;
  static constexpr bool swap_xy = false;
  static constexpr gpio_num_t backlight_io = GPIO_NUM_42;
  using DisplayDriver = espp::St7789;

  // touch
  static constexpr bool touch_swap_xy = true;
  static constexpr bool touch_invert_x = false;
  static constexpr bool touch_invert_y = true;
  static constexpr gpio_num_t touch_interrupt = GPIO_NUM_16;

  // trackball
  static constexpr gpio_num_t trackball_up = GPIO_NUM_15;
  static constexpr gpio_num_t trackball_down = GPIO_NUM_3;
  static constexpr gpio_num_t trackball_left = GPIO_NUM_1;
  static constexpr gpio_num_t trackball_right = GPIO_NUM_2;
  static constexpr gpio_num_t trackball_btn = GPIO_NUM_0;

  // uSD card
  static constexpr gpio_num_t sdcard_cs = GPIO_NUM_39;
  static constexpr gpio_num_t sdcard_mosi = GPIO_NUM_41;
  static constexpr gpio_num_t sdcard_miso = GPIO_NUM_38;
  static constexpr gpio_num_t sdcard_sclk = GPIO_NUM_40;

  // TODO: allow core id configuration
  I2c internal_i2c_{{.port = internal_i2c_port,
                     .sda_io_num = internal_i2c_sda,
                     .scl_io_num = internal_i2c_scl,
                     .sda_pullup_en = GPIO_PULLUP_ENABLE,
                     .scl_pullup_en = GPIO_PULLUP_ENABLE}};

  espp::Interrupt::PinConfig touch_interrupt_pin_{
      .gpio_num = touch_interrupt,
      .callback =
          [this](const auto &event) {
            if (update_gt911()) {
              if (touch_callback_) {
                touch_callback_(touchpad_data());
              }
            }
          },
      .active_level = espp::Interrupt::ActiveLevel::HIGH,
      .interrupt_type = espp::Interrupt::Type::RISING_EDGE};

  static constexpr auto trackball_interrupt_type = espp::Interrupt::Type::FALLING_EDGE;
  static constexpr auto trackball_filter_type = espp::Interrupt::FilterType::PIN_GLITCH_FILTER;
  espp::Interrupt::PinConfig trackball_up_interrupt_pin{
      .gpio_num = trackball_up,
      .callback = [this](const auto &event) { on_trackball_interrupt(event); },
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .interrupt_type = trackball_interrupt_type,
      .pullup_enabled = true,
      .filter_type = trackball_filter_type};
  espp::Interrupt::PinConfig trackball_down_interrupt_pin{
      .gpio_num = trackball_down,
      .callback = [this](const auto &event) { on_trackball_interrupt(event); },
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .interrupt_type = trackball_interrupt_type,
      .pullup_enabled = true,
      .filter_type = trackball_filter_type};
  espp::Interrupt::PinConfig trackball_left_interrupt_pin{
      .gpio_num = trackball_left,
      .callback = [this](const auto &event) { on_trackball_interrupt(event); },
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .interrupt_type = trackball_interrupt_type,
      .pullup_enabled = true,
      .filter_type = trackball_filter_type};
  espp::Interrupt::PinConfig trackball_right_interrupt_pin{
      .gpio_num = trackball_right,
      .callback = [this](const auto &event) { on_trackball_interrupt(event); },
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .interrupt_type = trackball_interrupt_type,
      .pullup_enabled = true,
      .filter_type = trackball_filter_type};
  espp::Interrupt::PinConfig trackball_btn_interrupt_pin{
      .gpio_num = trackball_btn,
      .callback = [this](const auto &event) { on_trackball_interrupt(event); },
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
      .pullup_enabled = true,
      .filter_type = trackball_filter_type};

  // we'll only add each interrupt pin if the initialize method is called
  espp::Interrupt interrupts_{
      {.interrupts = {},
       .event_queue_size = 50,
       .task_config = {.name = "t-deck interrupts",
                       .stack_size_bytes = CONFIG_TDECK_INTERRUPT_STACK_SIZE,
                       .priority = 20}}};

  // keyboard
  std::shared_ptr<TKeyboard> keyboard_{nullptr};

  // trackball
  std::atomic<int> trackball_sensitivity_{10};
  std::shared_ptr<PointerInput> pointer_input_{nullptr};
  std::recursive_mutex trackball_data_mutex_;
  espp::PointerData trackball_data_{};
  trackball_callback_t trackball_callback_{nullptr};

  // touch
  std::shared_ptr<Gt911> gt911_;
  std::shared_ptr<TouchpadInput> touchpad_input_;
  std::recursive_mutex touchpad_data_mutex_;
  espp::TouchpadData touchpad_data_;
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
}; // class TDeck
} // namespace espp
