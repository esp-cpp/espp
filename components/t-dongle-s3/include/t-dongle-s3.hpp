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
#include "led.hpp"
#include "led_strip.hpp"
#include "st7789.hpp"

namespace espp {
/// The TDongleS3 class provides an interface to the LilyGo T-Dongle-S3 ESP32-S3
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
/// \section t_dongle_s3_example Example
/// \snippet t_dongle_s3_example.cpp t-dongle-s3 example
class TDongleS3 : public BaseComponent {
public:
  /// Alias for the button callback function
  using button_callback_t = espp::Interrupt::event_callback_fn;

  /// Alias for the pixel type used by the T-Dongle-S3 display
  using Pixel = lv_color16_t;

  /// Mount point for the uSD card on the TDeck.
  static constexpr char mount_point[] = "/sdcard";

  /// @brief Access the singleton instance of the TDongleS3 class
  /// @return Reference to the singleton instance of the TDongleS3 class
  static TDongleS3 &get() {
    static TDongleS3 instance;
    return instance;
  }

  TDongleS3(const TDongleS3 &) = delete;
  TDongleS3 &operator=(const TDongleS3 &) = delete;
  TDongleS3(TDongleS3 &&) = delete;
  TDongleS3 &operator=(TDongleS3 &&) = delete;

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
  // RGB LED
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the RGB LED
  /// \return true if the RGB LED was successfully initialized, false otherwise
  bool initialize_led();

  /// Get the number of LEDs in the strip
  /// \return The number of LEDs in the strip
  static constexpr size_t num_leds() { return num_leds_; }

  /// Get a shared pointer to the RGB LED
  /// \return A shared pointer to the RGB LED
  std::shared_ptr<LedStrip> led() const;

  /// Set the color of the LED
  /// \param hsv The color of the LED in HSV format
  /// \param brightness The brightness of the LED as a percentage (0 - 100)
  /// \return true if the color was successfully set, false otherwise
  bool led(const Hsv &hsv, float brightness = 100.0f);

  /// Set the color of the LED
  /// \param rgb The color of the LED in RGB format
  /// \param brightness The brightness of the LED as a percentage (0 - 100)
  /// \return true if the color was successfully set, false otherwise
  bool led(const Rgb &rgb, float brightness = 100.0f);

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
  /// \note This function will only work after initialize_lcd() has been called
  void brightness(float brightness);

  /// Get the brightness of the backlight
  /// \return The brightness of the backlight as a percentage (0 - 100)
  /// \note This function will only work after initialize_lcd() has been called
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

protected:
  TDongleS3();
  void led_write(const uint8_t *data, size_t length);
  void lcd_wait_lines();

  // common:
  // APA102 LED
  static constexpr auto led_spi_num = SPI3_HOST;
  static constexpr auto led_mosi_io = GPIO_NUM_40;
  static constexpr auto led_sclk_io = GPIO_NUM_39;
  static constexpr auto led_clock_speed = 10 * 1000 * 1000; // 10 MHz
  static constexpr auto num_leds_ = 1;

  // LCD
  static constexpr size_t lcd_width_ = 160;
  static constexpr size_t lcd_height_ = 80;
  static constexpr size_t lcd_bytes_per_pixel = 2;
  static constexpr size_t frame_buffer_size = (((lcd_width_)*lcd_bytes_per_pixel) * lcd_height_);
  static constexpr int lcd_clock_speed = 40 * 1000 * 1000;
  static constexpr auto lcd_spi_num = SPI2_HOST;
  static constexpr gpio_num_t lcd_cs_io = GPIO_NUM_4;
  static constexpr gpio_num_t lcd_mosi_io = GPIO_NUM_3;
  static constexpr gpio_num_t lcd_sclk_io = GPIO_NUM_5;
  static constexpr gpio_num_t lcd_reset_io = GPIO_NUM_1;
  static constexpr gpio_num_t lcd_dc_io = GPIO_NUM_2;
  static constexpr int lcd_offset_x = 26;
  static constexpr int lcd_offset_y = 1;
  static constexpr bool backlight_value = false;
  static constexpr bool reset_value = false;
  static constexpr bool invert_colors = false;
  static constexpr bool swap_color_order = true;
  static constexpr auto rotation = espp::DisplayRotation::PORTRAIT;
  static constexpr bool swap_xy = false;
  static constexpr bool mirror_x = false;
  static constexpr bool mirror_y = false;
  static constexpr gpio_num_t backlight_io = GPIO_NUM_38;
  using DisplayDriver = espp::St7789;

  // button (boot button)
  static constexpr gpio_num_t button_io = GPIO_NUM_0; // active low

  // uSD card
  static constexpr gpio_num_t sdcard_d0 = GPIO_NUM_14;
  static constexpr gpio_num_t sdcard_d1 = GPIO_NUM_17;
  static constexpr gpio_num_t sdcard_d2 = GPIO_NUM_21;
  static constexpr gpio_num_t sdcard_d3 = GPIO_NUM_18;
  static constexpr gpio_num_t sdcard_clk = GPIO_NUM_12;
  static constexpr gpio_num_t sdcard_cmd = GPIO_NUM_16;

  // sdcard
  sdmmc_card_t *sdcard_{nullptr};

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
       .task_config = {.name = "t-dongle-s3 interrupts",
                       .stack_size_bytes = CONFIG_T_DONGLE_S3_INTERRUPT_STACK_SIZE}}};

  // button
  std::atomic<bool> button_initialized_{false};
  button_callback_t button_callback_{nullptr};

  // led
  std::shared_ptr<LedStrip> led_;
  spi_bus_config_t led_spi_bus_config_;
  spi_device_interface_config_t led_config_;
  spi_device_handle_t led_handle_{nullptr};

  // display
  std::shared_ptr<Display<Pixel>> display_;
  std::vector<Led::ChannelConfig> backlight_channel_configs_{};
  std::shared_ptr<Led> backlight_{};
  /// SPI bus for communication with the LCD
  spi_bus_config_t lcd_spi_bus_config_;
  spi_device_interface_config_t lcd_config_;
  spi_device_handle_t lcd_handle_{nullptr};
  static constexpr int spi_queue_size = 6;
  spi_transaction_t trans[spi_queue_size];
  std::atomic<int> num_queued_trans = 0;
  uint8_t *frame_buffer0_{nullptr};
  uint8_t *frame_buffer1_{nullptr};
}; // class TDongleS3
} // namespace espp
