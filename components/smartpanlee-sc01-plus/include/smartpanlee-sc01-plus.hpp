#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <vector>

#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <driver/spi_master.h>
#include <esp_lcd_io_i80.h>
#include <esp_lcd_panel_io.h>
#include <freertos/stream_buffer.h>
#include <hal/spi_types.h>
#include <sdmmc_cmd.h>

#include "base_component.hpp"
#include "display.hpp"
#include "ft5x06.hpp"
#include "i2c.hpp"
#include "interrupt.hpp"
#include "led.hpp"
#include "st7796.hpp"
#include "task.hpp"
#include "touchpad_input.hpp"

namespace espp {
/// The SmartPanleeSc01Plus class provides an interface to the Smart Panlee SC01
/// Plus development board.
///
/// The class provides access to the following features:
/// - ST7796 3.5-inch 320x480 display over an 8-bit Intel 8080 style bus
/// - FT5x06 capacitive touch controller
/// - PWM backlight control
/// - I2S audio playback for the onboard speaker path
/// - SPI microSD card mounting helpers
/// - Published I2S, RS-485, and expansion pin maps
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \section smartpanlee_sc01_plus_example Example
/// \snippet smartpanlee_sc01_plus_example.cpp smartpanlee sc01 plus example
class SmartPanleeSc01Plus : public BaseComponent {
public:
  /// Alias for the pixel type used by the display.
  using Pixel = lv_color16_t;
  /// Alias for the ST7796 display driver wrapper.
  using DisplayDriver = espp::St7796;
  /// Alias for the FT5x06-family capacitive touch controller.
  using TouchDriver = espp::Ft5x06;
  /// Alias for the touch data structure exposed to callers.
  using TouchpadData = espp::TouchpadData;
  /// Callback type invoked when fresh touch data is available.
  using touch_callback_t = std::function<void(const TouchpadData &)>;

  /// Published I2S pins for the SC01 Plus speaker interface.
  struct I2sPins {
    gpio_num_t mclk; ///< Optional master clock pin.
    gpio_num_t bclk; ///< I2S bit clock pin.
    gpio_num_t ws;   ///< I2S word-select / LRCLK pin.
    gpio_num_t dout; ///< I2S data-out pin.
    gpio_num_t din;  ///< Optional I2S data-in pin.
  };

  /// SPI microSD pin mapping published for the board.
  struct SdCardPins {
    gpio_num_t clk;  ///< SPI clock pin.
    gpio_num_t mosi; ///< SPI MOSI pin.
    gpio_num_t miso; ///< SPI MISO pin.
    gpio_num_t cs;   ///< SPI chip-select pin.
  };

  /// RS-485 UART control and data pins published for the board.
  struct Rs485Pins {
    gpio_num_t rts; ///< RS-485 direction-control pin.
    gpio_num_t rxd; ///< UART RX pin.
    gpio_num_t txd; ///< UART TX pin.
  };

  /// External expansion GPIOs published for the board.
  struct ExternalPins {
    gpio_num_t io1; ///< External GPIO 1.
    gpio_num_t io2; ///< External GPIO 2.
    gpio_num_t io3; ///< External GPIO 3.
    gpio_num_t io4; ///< External GPIO 4.
    gpio_num_t io5; ///< External GPIO 5.
    gpio_num_t io6; ///< External GPIO 6.
  };

  /// Configuration for mounting the optional microSD card.
  struct SdCardConfig {
    bool format_if_mount_failed{false};     ///< Format the card if mounting fails.
    int max_files{5};                       ///< Maximum number of simultaneously open files.
    size_t allocation_unit_size{16 * 1024}; ///< FAT allocation unit size in bytes.
  };

  /// Mount point used when the optional microSD card is mounted.
  static constexpr char mount_point[] = "/sdcard";

  /// Access the singleton instance.
  /// \return Reference to the singleton board-support instance.
  static SmartPanleeSc01Plus &get() {
    static SmartPanleeSc01Plus instance;
    return instance;
  }

  /// Deleted copy constructor.
  SmartPanleeSc01Plus(const SmartPanleeSc01Plus &) = delete;
  /// Deleted copy assignment operator.
  SmartPanleeSc01Plus &operator=(const SmartPanleeSc01Plus &) = delete;
  /// Deleted move constructor.
  SmartPanleeSc01Plus(SmartPanleeSc01Plus &&) = delete;
  /// Deleted move assignment operator.
  SmartPanleeSc01Plus &operator=(SmartPanleeSc01Plus &&) = delete;

  /// Get the published I2S pin mapping for the board.
  /// \return I2S pin mapping.
  static constexpr I2sPins i2s_pins() {
    return {GPIO_NUM_NC, GPIO_NUM_36, GPIO_NUM_35, GPIO_NUM_37, GPIO_NUM_NC};
  }

  /// Get the published SPI microSD pin mapping for the board.
  /// \return MicroSD pin mapping.
  static constexpr SdCardPins sd_card_pins() {
    return {GPIO_NUM_39, GPIO_NUM_40, GPIO_NUM_38, GPIO_NUM_41};
  }

  /// Get the published RS-485 pin mapping for the board.
  /// \return RS-485 pin mapping.
  static constexpr Rs485Pins rs485_pins() { return {GPIO_NUM_2, GPIO_NUM_1, GPIO_NUM_42}; }

  /// Get the published external expansion GPIO mapping for the board.
  /// \return External GPIO mapping.
  static constexpr ExternalPins external_pins() {
    return {GPIO_NUM_10, GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_21};
  }

  /// Get the internal I2C bus used for touch and board peripherals.
  /// \return Reference to the internal I2C instance.
  I2c &internal_i2c() { return internal_i2c_; }
  /// Get the interrupt manager used by the BSP.
  /// \return Reference to the interrupt manager.
  espp::Interrupt &interrupts() { return interrupts_; }

  /// Initialize the touch controller and optional interrupt-driven callback.
  /// \param callback Callback invoked when touch data changes.
  /// \return True if touch support was initialized, false otherwise.
  bool initialize_touch(const touch_callback_t &callback = nullptr);
  /// Get the LVGL touch input wrapper created by initialize_display().
  /// \return Shared pointer to the touchpad input wrapper, or nullptr if not initialized.
  std::shared_ptr<TouchpadInput> touchpad_input() const { return touchpad_input_; }
  /// Get the most recently cached touch data.
  /// \return Latest cached touchpad data.
  TouchpadData touchpad_data() const;
  /// Read the cached touch data using the signature expected by TouchpadInput.
  /// \param num_touch_points Filled with the number of active touch points.
  /// \param x Filled with the x coordinate of the primary touch point.
  /// \param y Filled with the y coordinate of the primary touch point.
  /// \param btn_state Filled with the button state for LVGL compatibility.
  void touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y, uint8_t *btn_state);
  /// Convert raw touch data into display-oriented coordinates.
  /// \param data Raw cached touch data.
  /// \return Touch data transformed for the current display rotation and inversion settings.
  TouchpadData touchpad_convert(const TouchpadData &data) const;

  /// Initialize the low-level LCD transport and backlight control.
  /// \return True if the LCD interface was initialized, false otherwise.
  bool initialize_lcd();
  /// Initialize the LVGL display wrapper.
  /// \param pixel_buffer_size Display buffer size in pixels.
  /// \return True if the display wrapper was initialized, false otherwise.
  bool initialize_display(size_t pixel_buffer_size = 320 * 40);
  /// Get the high-level LVGL display wrapper.
  /// \return Shared pointer to the display wrapper, or nullptr if not initialized.
  std::shared_ptr<Display<Pixel>> display() const { return display_; }
  /// Set the display backlight brightness.
  /// \param brightness Brightness percentage in the range [0, 100].
  void brightness(float brightness);
  /// Get the display backlight brightness.
  /// \return Brightness percentage in the range [0, 100].
  float brightness() const;

  /// Get the number of bytes per pixel used by the panel.
  /// \return Bytes per pixel.
  static constexpr size_t bytes_per_pixel() { return sizeof(Pixel); }
  /// Get the native display width.
  /// \return Width in pixels.
  static constexpr size_t display_width() { return lcd_width_; }
  /// Get the native display height.
  /// \return Height in pixels.
  static constexpr size_t display_height() { return lcd_height_; }
  /// Get the current display width after applying LVGL rotation.
  /// \return Rotated width in pixels.
  size_t rotated_display_width() const;
  /// Get the current display height after applying LVGL rotation.
  /// \return Rotated height in pixels.
  size_t rotated_display_height() const;

  /// Initialize the speaker audio path using the board's published I2S pins.
  /// \return True if audio playback support was initialized, false otherwise.
  bool initialize_audio();
  /// Initialize the speaker audio path using the specified sample rate.
  /// \param sample_rate Audio sample rate in Hz.
  /// \return True if audio playback support was initialized, false otherwise.
  bool initialize_audio(uint32_t sample_rate);
  /// Initialize the speaker audio path using a custom task configuration.
  /// \param sample_rate Audio sample rate in Hz.
  /// \param task_config Task configuration for the background audio pump.
  /// \return True if audio playback support was initialized, false otherwise.
  bool initialize_audio(uint32_t sample_rate, const espp::Task::BaseConfig &task_config);
  /// Set the audio sample rate.
  /// \param sample_rate Audio sample rate in Hz.
  void audio_sample_rate(uint32_t sample_rate);
  /// Get the active audio sample rate.
  /// \return Audio sample rate in Hz, or 0 if audio is not initialized.
  uint32_t audio_sample_rate() const;
  /// Get the internal playback buffer size.
  /// \return Audio buffer size in bytes.
  size_t audio_buffer_size() const;
  /// Mute or unmute speaker playback.
  /// \param mute True to mute playback, false to unmute it.
  void mute(bool mute);
  /// Query whether speaker playback is muted.
  /// \return True if playback is muted.
  bool is_muted() const;
  /// Set the software playback volume.
  /// \param volume Volume percentage in the range [0, 100].
  void volume(float volume);
  /// Get the software playback volume.
  /// \return Volume percentage in the range [0, 100].
  float volume() const;
  /// Queue raw PCM audio bytes for playback.
  /// \param data Audio payload to play.
  void play_audio(std::span<const uint8_t> data);
  /// Queue raw PCM audio bytes for playback.
  /// \param data Pointer to PCM audio bytes.
  /// \param num_bytes Number of bytes to play.
  void play_audio(const uint8_t *data, uint32_t num_bytes);

  /// Initialize and mount the optional microSD card with default settings.
  /// \return True if the card was mounted successfully, false otherwise.
  bool initialize_sdcard();
  /// Initialize and mount the optional microSD card with custom settings.
  /// \param config Mount configuration.
  /// \return True if the card was mounted successfully, false otherwise.
  bool initialize_sdcard(const SdCardConfig &config);
  /// Check whether the microSD card is currently mounted.
  /// \return True if the card is mounted and available.
  bool is_sd_card_available() const;
  /// Query mounted microSD capacity and free space.
  /// \param size_mb Optional output for total capacity in megabytes.
  /// \param free_mb Optional output for free space in megabytes.
  /// \return True if card information was retrieved successfully, false otherwise.
  bool get_sd_card_info(uint32_t *size_mb, uint32_t *free_mb) const;

protected:
  SmartPanleeSc01Plus();

  static bool panel_io_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                                        esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
  static espp::Task::BaseConfig default_audio_task_config();
  bool initialize_i2s(uint32_t default_audio_rate);
  bool audio_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);
  bool update_touch();
  void write_command(uint8_t command, std::span<const uint8_t> parameters, uint32_t user_data);
  void write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data, uint32_t user_data);

  static constexpr auto internal_i2c_port = I2C_NUM_1;
  static constexpr auto internal_i2c_clock_speed = 400 * 1000;
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_6;
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_5;

  static constexpr size_t lcd_width_ = 320;
  static constexpr size_t lcd_height_ = 480;
  static constexpr int lcd_clock_speed_hz = 40 * 1000 * 1000;
  static constexpr size_t lcd_max_transfer_bytes = lcd_width_ * 40 * sizeof(Pixel);
  static constexpr gpio_num_t lcd_reset_io = GPIO_NUM_4;
  static constexpr gpio_num_t lcd_backlight_io = GPIO_NUM_45;
  static constexpr gpio_num_t lcd_dc_io = GPIO_NUM_0;
  static constexpr gpio_num_t lcd_wr_io = GPIO_NUM_47;
  static constexpr gpio_num_t lcd_cs_io = GPIO_NUM_NC;
  static constexpr gpio_num_t lcd_d0_io = GPIO_NUM_9;
  static constexpr gpio_num_t lcd_d1_io = GPIO_NUM_46;
  static constexpr gpio_num_t lcd_d2_io = GPIO_NUM_3;
  static constexpr gpio_num_t lcd_d3_io = GPIO_NUM_8;
  static constexpr gpio_num_t lcd_d4_io = GPIO_NUM_18;
  static constexpr gpio_num_t lcd_d5_io = GPIO_NUM_17;
  static constexpr gpio_num_t lcd_d6_io = GPIO_NUM_16;
  static constexpr gpio_num_t lcd_d7_io = GPIO_NUM_15;
  static constexpr bool backlight_value = true;
  static constexpr bool invert_colors = true;
  static constexpr auto rotation = espp::DisplayRotation::LANDSCAPE;
  static constexpr bool mirror_x = false;
  static constexpr bool mirror_y = false;
  static constexpr bool swap_xy = false;
  static constexpr bool swap_color_order = true;

  static constexpr bool touch_swap_xy = false;
  static constexpr bool touch_invert_x = false;
  static constexpr bool touch_invert_y = false;
  static constexpr gpio_num_t touch_interrupt = GPIO_NUM_7;

  static constexpr auto i2s_port = I2S_NUM_0;
  static constexpr int audio_num_channels = 2;
  static constexpr int audio_num_bytes_per_channel = 2;
  static constexpr int audio_update_frequency = 60;
  static constexpr int calc_audio_buffer_size(int sample_rate) {
    return sample_rate * audio_num_channels * audio_num_bytes_per_channel / audio_update_frequency;
  }

  I2c internal_i2c_{I2c::Config{.port = internal_i2c_port,
                                .sda_io_num = internal_i2c_sda,
                                .scl_io_num = internal_i2c_scl,
                                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                                .clk_speed = internal_i2c_clock_speed}};

  espp::Interrupt::PinConfig touch_interrupt_pin_{.gpio_num = touch_interrupt,
                                                  .callback =
                                                      [this](const auto &event) {
                                                        if (!event.active) {
                                                          return;
                                                        }
                                                        if (update_touch() && touch_callback_) {
                                                          touch_callback_(touchpad_data());
                                                        }
                                                      },
                                                  .active_level = espp::Interrupt::ActiveLevel::LOW,
                                                  .interrupt_type =
                                                      espp::Interrupt::Type::FALLING_EDGE,
                                                  .pullup_enabled = true};

  espp::Interrupt interrupts_{
      {.interrupts = {},
       .task_config = {.name = "sc01+ interrupts",
                       .stack_size_bytes = CONFIG_SMARTPANLEE_SC01_PLUS_INTERRUPT_STACK_SIZE}}};

  std::shared_ptr<TouchDriver> touch_driver_;
  std::shared_ptr<TouchpadInput> touchpad_input_;
  mutable std::recursive_mutex touchpad_data_mutex_;
  TouchpadData touchpad_data_;
  touch_callback_t touch_callback_{nullptr};

  std::shared_ptr<Display<Pixel>> display_;
  std::shared_ptr<Led> backlight_;
  std::vector<Led::ChannelConfig> backlight_channel_configs_;
  esp_lcd_i80_bus_handle_t lcd_bus_{nullptr};
  esp_lcd_panel_io_handle_t panel_io_{nullptr};

  std::atomic<bool> audio_initialized_{false};
  std::atomic<float> volume_{50.0f};
  std::atomic<bool> mute_{false};
  std::unique_ptr<espp::Task> audio_task_{nullptr};
  i2s_chan_handle_t audio_tx_handle_{nullptr};
  StreamBufferHandle_t audio_tx_stream_{nullptr};
  i2s_std_config_t audio_std_cfg_{};
  std::vector<uint8_t> audio_tx_buffer_;

  bool sd_card_initialized_{false};
  sdmmc_card_t *sdcard_{nullptr};
};
} // namespace espp
