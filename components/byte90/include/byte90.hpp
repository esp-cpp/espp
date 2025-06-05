#pragma once

#include <memory>
#include <string>
#include <vector>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <hal/spi_types.h>

#include "adxl345.hpp"
#include "base_component.hpp"
#include "i2c.hpp"
#include "interrupt.hpp"
#include "ssd1351.hpp"

namespace espp {
/// The Byte90 class provides an interface to the Byte90 ESP32-S3 development
/// board.
///
/// The class provides access to the following features:
/// - Display
/// - Interrupts
/// - I2C
/// - Accelerometer (ADXL345)
/// - Button
///
/// The class is a singleton and can be accessed using the get() method.
///
/// For more information, see
/// https://github.com/alxv2016/Byte90-alxvlabs
///
/// **Pin Reference Table:**
///
/// | XIAO Pin | GPIO | Function | Component |
/// |----------|------|----------|-----------|
/// | D0 | GPIO1 | RST | Display Reset |
/// | D1 | GPIO2 | INT | ADXL345 Interrupt |
/// | A3 | GPIO4 | INPUT | Button (with pull-up) |
/// | D4 | GPIO5 | SDA | ADXL345 I2C Data |
/// | D5 | GPIO6 | SCL | ADXL345 I2C Clock |
/// | D8 | GPIO7 | SCK | Display SPI Clock |
/// | D10 | GPIO9 | MOSI | Display SPI Data |
/// | D6 | GPIO43 | DC | Display Data/Command |
/// | D7 | GPIO44 | CS | Display Chip Select |
///
/// \section byte90_example Example
/// \snippet byte90_example.cpp byte90 example
class Byte90 : public BaseComponent {
public:
  /// Alias for the button callback function
  using button_callback_t = espp::Interrupt::event_callback_fn;

  /// Alias for the Accelerometer type
  using Accelerometer = espp::Adxl345;

  /// Alias for the Accelerometer data type
  using AccelerometerData = Accelerometer::Data;

  /// Alias for the pixel type used by the Byte90 display
  using Pixel = lv_color16_t;

  /// Alias for the display driver used by the Byte90 display
  using DisplayDriver = espp::Ssd1351;

  /// Alias for an accelerometer interrupt callback
  using accel_callback_t =
      std::function<void(const std::chrono::high_resolution_clock::time_point &timestamp,
                         const std::vector<AccelerometerData> &data)>;

  /// @brief Access the singleton instance of the Byte90 class
  /// @return Reference to the singleton instance of the Byte90 class
  static Byte90 &get() {
    static Byte90 instance;
    return instance;
  }

  Byte90(const Byte90 &) = delete;
  Byte90 &operator=(const Byte90 &) = delete;
  Byte90(Byte90 &&) = delete;
  Byte90 &operator=(Byte90 &&) = delete;

  /// Get a reference to the internal I2C bus
  /// \return A reference to the internal I2C bus
  /// \note The internal I2C bus is used for the accelerometer
  I2c &internal_i2c() { return internal_i2c_; }

  /// Get a reference to the interrupts
  /// \return A reference to the interrupts
  espp::Interrupt &interrupts() { return interrupts_; }

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
  // Accelerometer (ADXL345)
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the accelerometer
  /// \param callback The callback function to call when the accelerometer
  ///        interrupt occurs
  /// \return true if the accelerometer was successfully initialized, false
  ///         otherwise
  /// \note This will configure the accelerometer interrupt pin which will
  ///       automatically call the accel callback function when the accel
  ///       interrupt occurs
  bool initialize_accelerometer(const accel_callback_t &callback = nullptr);

  /// Get the Accelerometer shared pointer
  /// \return A shared pointer to the Accelerometer instance
  std::shared_ptr<Accelerometer> accelerometer() const { return accelerometer_; }

  /// Get the most recent accelerometer data
  /// \return The most recent accelerometer data
  std::vector<AccelerometerData> accelerometer_data() {
    std::lock_guard<std::recursive_mutex> lock(accel_data_mutex_);
    return accelerometer_data_;
  }

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

protected:
  Byte90();
  bool init_spi_bus();
  void lcd_wait_lines();

  // common:
  // internal i2c (adxl345)
  static constexpr auto internal_i2c_port = I2C_NUM_0;
  static constexpr auto internal_i2c_clock_speed = 400 * 1000;
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_5;
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_6;

  // button
  static constexpr gpio_num_t button_io = GPIO_NUM_4; // A3, active low, with pullup

  // accelerometer
  static constexpr gpio_num_t accelerometer_interrupt = GPIO_NUM_2; // D1

  // SPI (LCD)
  static constexpr gpio_num_t spi_mosi_io = GPIO_NUM_9; // D10
  static constexpr gpio_num_t spi_sclk_io = GPIO_NUM_7; // D8
  static constexpr auto spi_num = SPI3_HOST;

  // LCD
  static constexpr size_t lcd_width_ = 128;
  static constexpr size_t lcd_height_ = 128;
  static constexpr size_t lcd_bytes_per_pixel = 2;
  static constexpr size_t frame_buffer_size = (((lcd_width_)*lcd_bytes_per_pixel) * lcd_height_);
  static constexpr int lcd_clock_speed = 20 * 1000 * 1000;
  static constexpr gpio_num_t lcd_cs_io = GPIO_NUM_44;   // D7
  static constexpr gpio_num_t lcd_reset_io = GPIO_NUM_1; // D0
  static constexpr gpio_num_t lcd_dc_io = GPIO_NUM_43;   // D6
  static constexpr bool reset_value = false;
  static constexpr bool invert_colors = false;
  static constexpr auto rotation = espp::DisplayRotation::LANDSCAPE;
  static constexpr bool mirror_x = false;
  static constexpr bool mirror_y = true;
  static constexpr bool mirror_portrait = false;
  static constexpr bool swap_xy = false;

  // TODO: allow core id configuration
  I2c internal_i2c_{{.port = internal_i2c_port,
                     .sda_io_num = internal_i2c_sda,
                     .scl_io_num = internal_i2c_scl,
                     .sda_pullup_en = GPIO_PULLUP_ENABLE,
                     .scl_pullup_en = GPIO_PULLUP_ENABLE}};

  // spi bus shared between sdcard and lcd
  std::atomic<bool> spi_bus_initialized_{false};

  espp::Interrupt::PinConfig button_interrupt_pin_{
      .gpio_num = button_io,
      .callback =
          [this](const auto &event) {
            if (button_callback_) {
              button_callback_(event);
            }
          },
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .interrupt_type = espp::Interrupt::Type::FALLING_EDGE,
      .pullup_enabled = true,
      .filter_type = espp::Interrupt::FilterType::PIN_GLITCH_FILTER,
  };

  espp::Interrupt::PinConfig accelerometer_interrupt_pin_{
      .gpio_num = accelerometer_interrupt,
      .callback =
          [this](const auto &event) {
            std::error_code ec;
            // clear the interrupt source
            [[maybe_unused]] auto interrupt_status = accelerometer_->get_interrupt_source(ec);
            if (ec) {
              logger_.error("Error getting interrupt source: {}", ec.message());
              return;
            }
            auto now = std::chrono::high_resolution_clock::now();
            // we got a watermark interrupt, read the data and print it
            auto data_vec = accelerometer_->read_all(ec);
            if (ec) {
              logger_.error("Error reading ADXL345: {}", ec.message());
              return;
            }
            // store the data in a thread-safe manner
            {
              std::lock_guard<std::recursive_mutex> lock(accel_data_mutex_);
              accelerometer_data_ = data_vec;
            }
            // call the callback if it is set
            if (accel_callback_) {
              accel_callback_(now, data_vec);
            }
          },
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .interrupt_type = espp::Interrupt::Type::FALLING_EDGE,
      .pullup_enabled = true,
      .pulldown_enabled = false,
      .filter_type = espp::Interrupt::FilterType::PIN_GLITCH_FILTER};

  // we'll only add each interrupt pin if the initialize method is called
  espp::Interrupt interrupts_{
      {.interrupts = {},
       .event_queue_size = 50,
       .task_config = {.name = "byte90 interrupts",
                       .stack_size_bytes = CONFIG_BYTE90_INTERRUPT_STACK_SIZE,
                       .priority = 20}}};

  // button
  std::atomic<bool> button_initialized_{false};
  button_callback_t button_callback_{nullptr};

  // accelerometer
  std::shared_ptr<Accelerometer> accelerometer_{nullptr};
  accel_callback_t accel_callback_{nullptr};
  std::recursive_mutex accel_data_mutex_;
  std::vector<AccelerometerData> accelerometer_data_{};

  // display
  std::shared_ptr<Display<Pixel>> display_;
  /// SPI bus for communication with the LCD
  spi_device_interface_config_t lcd_config_;
  spi_device_handle_t lcd_handle_{nullptr};
  static constexpr int spi_queue_size = 6;
  spi_transaction_t trans[spi_queue_size];
  std::atomic<int> num_queued_trans = 0;
  uint8_t *frame_buffer0_{nullptr};
  uint8_t *frame_buffer1_{nullptr};
}; // class Byte90
} // namespace espp
