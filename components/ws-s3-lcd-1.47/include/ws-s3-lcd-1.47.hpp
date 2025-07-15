#pragma once

#include <memory>
#include <string>
#include <vector>

#include <esp_err.h>
#include <esp_partition.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>

#include <driver/gpio.h>
#include <driver/sdmmc_host.h>
#include <driver/spi_master.h>
#include <hal/spi_types.h>

#include "base_component.hpp"
#include "interrupt.hpp"
#include "neopixel.hpp"
#include "st7789.hpp"

namespace espp {
/// The WsS3Lcd147 class provides an interface to the Waveshare ESP32-S3-LCD-1.47
/// development board.
///
/// The class provides access to the following features:
/// - Button (boot button)
/// - Display
/// - RGB LED
/// - micro-SD (uSD) card
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \section ws_s3_lcd_1.47_example Example
/// \snippet ws_s3_lcd_1.47_example.cpp ws-s3-lcd-1.47 example
class WsS3Lcd147 : public BaseComponent {
public:
  /// Alias for the button callback function
  using button_callback_t = espp::Interrupt::event_callback_fn;

  /// Alias for the pixel type used by the display
  using Pixel = lv_color16_t;

  /// Mount point for the uSD card on the TDeck.
  static constexpr char mount_point[] = "/sdcard";

  /// @brief Access the singleton instance of the WsS3Lcd147 class
  /// @return Reference to the singleton instance of the WsS3Lcd147 class
  static WsS3Lcd147 &get() {
    static WsS3Lcd147 instance;
    return instance;
  }

  WsS3Lcd147(const WsS3Lcd147 &) = delete;
  WsS3Lcd147 &operator=(const WsS3Lcd147 &) = delete;
  WsS3Lcd147(WsS3Lcd147 &&) = delete;
  WsS3Lcd147 &operator=(WsS3Lcd147 &&) = delete;

  /// Get a reference to the interrupts
  /// \return A reference to the interrupts
  espp::Interrupt &interrupts();

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

  /////////////////////////////////////////////////////////////////////////////
  // uSD Card
  /////////////////////////////////////////////////////////////////////////////

  /// Configuration for the uSD card
  struct SdCardConfig {
    bool format_if_mount_failed = false;    ///< Format the uSD card if mount failed
    int max_files = 5;                      ///< The maximum number of files to open at once
    size_t allocation_unit_size = 2 * 1024; ///< The allocation unit size in bytes
  };

  /// Initialize the uSD card
  /// \param config The configuration for the uSD card
  /// \return True if the uSD card was initialized properly.
  bool initialize_sdcard(const SdCardConfig &config);

  /// Get the uSD card
  /// \return A pointer to the uSD card
  /// \note The uSD card is only available if it was successfully initialized
  ///       and the mount point is valid
  sdmmc_card_t *sdcard() const { return sdcard_; }

  /////////////////////////////////////////////////////////////////////////////
  // LED
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the RGB LED
  /// \return true if the LED was successfully initialized, false otherwise
  bool initialize_led();

  /// Get a shared pointer to the RGB LED
  /// \return A shared pointer to the RGB LED
  std::shared_ptr<Neopixel> led() const { return led_; }

  /// Set the color of the LED
  /// \param hsv The color of the LED in HSV format
  /// \return true if the color was successfully set, false otherwise
  bool led(const Hsv &hsv);

  /// Set the color of the LED
  /// \param rgb The color of the LED in RGB format
  /// \return true if the color was successfully set, false otherwise
  bool led(const Rgb &rgb);

protected:
  WsS3Lcd147();
  void lcd_wait_lines();

  // LCD
  static constexpr size_t lcd_width_ = 320;
  static constexpr size_t lcd_height_ = 172;
  static constexpr size_t lcd_bytes_per_pixel = 2;
  static constexpr size_t frame_buffer_size = (((lcd_width_)*lcd_bytes_per_pixel) * lcd_height_);
  static constexpr int lcd_clock_speed = 80 * 1000 * 1000;
  static constexpr auto lcd_spi_num = SPI2_HOST;
  static constexpr gpio_num_t lcd_cs_io = GPIO_NUM_42;
  static constexpr gpio_num_t lcd_mosi_io = GPIO_NUM_45;
  static constexpr gpio_num_t lcd_sclk_io = GPIO_NUM_40;
  static constexpr gpio_num_t lcd_reset_io = GPIO_NUM_39;
  static constexpr gpio_num_t lcd_dc_io = GPIO_NUM_41;
  static constexpr int lcd_offset_x = 0;
  static constexpr int lcd_offset_y = 34;
  static constexpr bool backlight_value = true;
  static constexpr bool reset_value = false;
  static constexpr bool invert_colors = false;
  static constexpr bool swap_color_order = false;
  static constexpr auto rotation = espp::DisplayRotation::LANDSCAPE;
  static constexpr bool swap_xy = false;
  static constexpr bool mirror_x = false;
  static constexpr bool mirror_y = false;
  static constexpr gpio_num_t backlight_io = GPIO_NUM_48;
  using DisplayDriver = espp::St7789;

  // button (boot button)
  static constexpr gpio_num_t button_io = GPIO_NUM_0; // active low

  // uSD card
  static constexpr gpio_num_t sdcard_d0 = GPIO_NUM_16;
  static constexpr gpio_num_t sdcard_d1 = GPIO_NUM_18;
  static constexpr gpio_num_t sdcard_d2 = GPIO_NUM_17;
  static constexpr gpio_num_t sdcard_d3 = GPIO_NUM_21;
  static constexpr gpio_num_t sdcard_clk = GPIO_NUM_14;
  static constexpr gpio_num_t sdcard_cmd = GPIO_NUM_15;

  // RGB LED
  static constexpr gpio_num_t rgb_led_io = GPIO_NUM_38;

  // sdcard
  sdmmc_card_t *sdcard_{nullptr};
  std::shared_ptr<Neopixel> led_{nullptr};

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
      .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
      .pullup_enabled = true};

  // we'll only add each interrupt pin if the initialize method is called
  espp::Interrupt interrupts_{
      {.interrupts = {},
       .task_config = {.name = "ws-s3-lcd-1.47 interrupts",
                       .stack_size_bytes = CONFIG_WS_S3_LCD_1_47_INTERRUPT_STACK_SIZE}}};

  // button
  std::atomic<bool> button_initialized_{false};
  button_callback_t button_callback_{nullptr};

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
}; // class WsS3Lcd147
} // namespace espp
