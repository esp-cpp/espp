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
#include <hal/spi_ll.h>
#include <hal/spi_types.h>

#include "base_component.hpp"
#include "interrupt.hpp"
#include "led.hpp"
#include "spi.hpp"
#include "st7789.hpp"

namespace espp {
/// The WsS3Geek class provides an interface to the Waveshare ESP32-S3-GEEK
/// development board.
///
/// The class provides access to the following features:
/// - Button (boot button)
/// - Display
/// - micro-SD (uSD) card
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \section ws_s3_geek_example Example
/// \snippet ws_s3_geek_example.cpp ws-s3-geek example
class WsS3Geek : public BaseComponent {
public:
  /// Alias for the button callback function
  using button_callback_t = espp::Interrupt::event_callback_fn;

  /// Alias for the pixel type used by the display
  using Pixel = lv_color16_t;

  /// Alias for the display driver
  using DisplayDriver = espp::St7789;

  /// Maximum number of bytes that can be transferred in a single SPI
  /// transaction to the Display. 32k on the ESP32-S3.
  static constexpr size_t SPI_MAX_TRANSFER_BYTES = SPI_LL_DMA_MAX_BIT_LEN / 8;

  /// Mount point for the uSD card.
  static constexpr char mount_point[] = "/sdcard";

  /// @brief Access the singleton instance of the WsS3Geek class
  /// @return Reference to the singleton instance of the WsS3Geek class
  static WsS3Geek &get() {
    static WsS3Geek instance;
    return instance;
  }

  WsS3Geek(const WsS3Geek &) = delete;
  WsS3Geek &operator=(const WsS3Geek &) = delete;
  WsS3Geek(WsS3Geek &&) = delete;
  WsS3Geek &operator=(WsS3Geek &&) = delete;

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

  /// Get the display width in pixels, according to the current orientation
  /// \return The display width in pixels, according to the current orientation
  size_t rotated_display_width() const;

  /// Get the display height in pixels, according to the current orientation
  /// \return The display height in pixels, according to the current orientation
  size_t rotated_display_height() const;

  /// Get the GPIO pin for the LCD data/command signal
  /// \return The GPIO pin for the LCD data/command signal
  static constexpr auto get_lcd_dc_gpio() { return lcd_dc_io; }

  /// Get a shared pointer to the display
  /// \return A shared pointer to the display
  std::shared_ptr<Display<Pixel>> display() const;

  /// Get a shared pointer to the low-level display driver
  /// \return A shared pointer to the display driver
  const std::shared_ptr<DisplayDriver> &display_driver() const { return display_driver_; }

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
  /// \param user_data User data to pass to the SPI transaction callback
  /// \note This method queues the panel transfer asynchronously and may return
  ///       before the write has completed.
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
  WsS3Geek();
  void lcd_wait_lines();

  // LCD
  static constexpr size_t lcd_width_ = 240;
  static constexpr size_t lcd_height_ = 135;
  static constexpr size_t lcd_bytes_per_pixel = 2;
  static constexpr size_t frame_buffer_size = (((lcd_width_)*lcd_bytes_per_pixel) * lcd_height_);
  static constexpr int lcd_clock_speed = 80 * 1000 * 1000;
  static constexpr auto lcd_spi_num = SPI2_HOST;
  static constexpr gpio_num_t lcd_cs_io = GPIO_NUM_10;
  static constexpr gpio_num_t lcd_mosi_io = GPIO_NUM_11;
  static constexpr gpio_num_t lcd_sclk_io = GPIO_NUM_12;
  static constexpr gpio_num_t lcd_reset_io = GPIO_NUM_9;
  static constexpr gpio_num_t lcd_dc_io = GPIO_NUM_8;
  static constexpr int lcd_offset_x = 40;
  static constexpr int lcd_offset_y = 52;
  static constexpr bool backlight_value = true;
  static constexpr bool reset_value = false;
  static constexpr bool invert_colors = false;
  static constexpr bool swap_color_order = false;
  static constexpr auto rotation = espp::DisplayRotation::LANDSCAPE_INVERTED;
  static constexpr bool swap_xy = false;
  static constexpr bool mirror_x = false;
  static constexpr bool mirror_y = false;
  static constexpr gpio_num_t backlight_io = GPIO_NUM_7;

  // button (boot button)
  static constexpr gpio_num_t button_io = GPIO_NUM_0; // active low

  // uSD card (sdmmc)
  static constexpr gpio_num_t sdcard_d0 = GPIO_NUM_37;
  static constexpr gpio_num_t sdcard_d1 = GPIO_NUM_33;
  static constexpr gpio_num_t sdcard_d2 = GPIO_NUM_38;
  static constexpr gpio_num_t sdcard_d3 = GPIO_NUM_34;
  static constexpr gpio_num_t sdcard_clk = GPIO_NUM_36;
  static constexpr gpio_num_t sdcard_cmd = GPIO_NUM_35;
  // for SPI
  static constexpr auto sdcard_spi_num = SPI3_HOST;
  static constexpr gpio_num_t sdcard_miso = GPIO_NUM_37; // same as d0
  static constexpr gpio_num_t sdcard_cs = GPIO_NUM_34;   // same as d3
  static constexpr gpio_num_t sdcard_mosi = GPIO_NUM_35; // same as cmd

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
       .task_config = {.name = "ws-s3-geek interrupts",
                       .stack_size_bytes = CONFIG_WS_S3_GEEK_INTERRUPT_STACK_SIZE,
                       .priority = CONFIG_WS_S3_GEEK_INTERRUPT_PRIORITY,
                       .core_id = CONFIG_WS_S3_GEEK_INTERRUPT_CORE_ID}}};

  // button
  std::atomic<bool> button_initialized_{false};
  button_callback_t button_callback_{nullptr};

  // display
  std::shared_ptr<Display<Pixel>> display_;
  std::shared_ptr<DisplayDriver> display_driver_{static_cast<DisplayDriver *>(nullptr)};
  std::vector<Led::ChannelConfig> backlight_channel_configs_{};
  std::shared_ptr<Led> backlight_{};
  static constexpr int spi_queue_size = 6;
  std::unique_ptr<Spi> lcd_spi_;
  std::unique_ptr<SpiPanelIo> lcd_;
}; // class WsS3Geek
} // namespace espp
