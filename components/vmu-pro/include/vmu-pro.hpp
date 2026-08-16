#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <esp_err.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>

#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <driver/sdmmc_host.h>
#include <driver/spi_master.h>
#include <hal/spi_ll.h>
#include <hal/spi_types.h>

#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

#include "base_component.hpp"
#include "interrupt.hpp"
#include "led.hpp"
#include "spi.hpp"
#include "st7789.hpp"
#include "task.hpp"

namespace espp {
/// The VmuPro class provides an interface to the 8BitMods VMU Pro, an
/// ESP32-S3 based replacement for the Sega Dreamcast Visual Memory Unit
/// (VMU) which doubles as a standalone handheld.
///
/// The class provides access to the following features:
/// - Display (1.5" 240x240 IPS TFT)
/// - Buttons (D-pad, A, B, Mode, Power, Bottom)
/// - Audio (mono speaker via I2S amplifier)
/// - micro-SD (uSD) card
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \warning The GPIO assignments in this BSP are UNVERIFIED placeholders.
///          The VMU Pro's schematic is not publicly available, so the pin
///          numbers below must be corrected against real hardware or vendor
///          documentation before use. The peripheral set (display size and
///          color format, button list, audio capabilities, uSD card) comes
///          from the official VMU Pro SDK and is accurate.
///
/// \section vmu_pro_example Example
/// \snippet vmu_pro_example.cpp vmu-pro example
class VmuPro : public BaseComponent {
public:
  /// The buttons on the VMU Pro
  enum class Button : uint8_t {
    DPAD_UP = 0, ///< D-pad up
    DPAD_DOWN,   ///< D-pad down
    DPAD_LEFT,   ///< D-pad left
    DPAD_RIGHT,  ///< D-pad right
    A,           ///< A button
    B,           ///< B button
    MODE,        ///< Mode button
    POWER,       ///< Power button
    BOTTOM,      ///< Bottom (sleep) button
  };

  /// Number of buttons on the VMU Pro
  static constexpr size_t NUM_BUTTONS = 9;

  /// Alias for the button callback function. Called with the button that
  /// changed state and whether it is now pressed.
  using button_callback_t = std::function<void(Button button, bool pressed)>;

  /// Alias for the pixel type used by the display
  using Pixel = lv_color16_t;

  /// Alias for the display driver
  using DisplayDriver = espp::St7789;

  /// Maximum number of bytes that can be transferred in a single SPI
  /// transaction to the Display. 32k on the ESP32-S3.
  static constexpr size_t SPI_MAX_TRANSFER_BYTES = SPI_LL_DMA_MAX_BIT_LEN / 8;

  /// Mount point for the uSD card.
  static constexpr char mount_point[] = "/sdcard";

  /// @brief Access the singleton instance of the VmuPro class
  /// @return Reference to the singleton instance of the VmuPro class
  static VmuPro &get() {
    static VmuPro instance;
    return instance;
  }

  VmuPro(const VmuPro &) = delete;
  VmuPro &operator=(const VmuPro &) = delete;
  VmuPro(VmuPro &&) = delete;
  VmuPro &operator=(VmuPro &&) = delete;

  /// Get a reference to the interrupts
  /// \return A reference to the interrupts
  espp::Interrupt &interrupts();

  /////////////////////////////////////////////////////////////////////////////
  // Buttons
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the buttons
  /// \param callback The callback function to call when a button changes
  ///        state
  /// \return true if the buttons were successfully initialized, false
  ///         otherwise
  bool initialize_buttons(const button_callback_t &callback = nullptr);

  /// Get the state of a button
  /// \param button The button to get the state of
  /// \return The button state (true = button pressed, false = button
  ///         released)
  bool button_state(Button button) const;

  /// Get the name of a button
  /// \param button The button to get the name of
  /// \return The name of the button
  static const char *button_name(Button button) {
    switch (button) {
    case Button::DPAD_UP:
      return "Up";
    case Button::DPAD_DOWN:
      return "Down";
    case Button::DPAD_LEFT:
      return "Left";
    case Button::DPAD_RIGHT:
      return "Right";
    case Button::A:
      return "A";
    case Button::B:
      return "B";
    case Button::MODE:
      return "Mode";
    case Button::POWER:
      return "Power";
    case Button::BOTTOM:
      return "Bottom";
    default:
      return "Unknown";
    }
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
  // Audio
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the sound subsystem (I2S amplifier driving the mono speaker)
  /// \param default_audio_rate The default sample rate for the audio, in Hz
  /// \param task_config The configuration for the audio task
  /// \return true if the sound subsystem was successfully initialized, false
  ///         otherwise
  bool
  initialize_sound(uint32_t default_audio_rate = 44100,
                   const espp::Task::BaseConfig &task_config = {
                       .name = "audio", .stack_size_bytes = 4096, .priority = 19, .core_id = 1});

  /// Enable or disable the audio amplifier
  /// \param enable true to enable the amplifier, false to disable it
  void enable_sound(bool enable);

  /// Get the audio sample rate
  /// \return The audio sample rate, in Hz
  uint32_t audio_sample_rate() const;

  /// Set the audio sample rate
  /// \param sample_rate The audio sample rate, in Hz
  void audio_sample_rate(uint32_t sample_rate);

  /// Get the audio buffer size
  /// \return The audio buffer size, in bytes
  size_t audio_buffer_size() const;

  /// Mute or unmute the audio
  /// \param mute true to mute the audio, false to unmute it
  void mute(bool mute);

  /// Check if the audio is muted
  /// \return true if the audio is muted, false otherwise
  bool is_muted() const;

  /// Set the volume of the audio
  /// \param volume The volume as a percentage (0 - 100)
  /// \note The VMU Pro has no hardware volume control, so the volume is
  ///       applied in software when the samples are written to the I2S
  ///       peripheral
  void volume(float volume);

  /// Get the volume of the audio
  /// \return The volume as a percentage (0 - 100)
  float volume() const;

  /// Play the audio data
  /// \param data The audio data to play (16-bit signed mono samples)
  /// \note This function is non-blocking and queues the data for the audio
  ///       task to play
  void play_audio(const std::vector<uint8_t> &data);

  /// Play the audio data
  /// \param data The audio data to play (16-bit signed mono samples)
  /// \param num_bytes The number of bytes to play
  /// \note This function is non-blocking and queues the data for the audio
  ///       task to play
  void play_audio(const uint8_t *data, uint32_t num_bytes);

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
  VmuPro();
  void lcd_wait_lines();
  bool initialize_i2s(uint32_t default_audio_rate);
  bool audio_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);

  //////////////////////////////////////////////////////////////////////////
  // NOTE: the GPIO assignments below are UNVERIFIED placeholders! The VMU
  // Pro's schematic is not publicly available; these values were chosen to
  // be valid ESP32-S3 pins so the component compiles, and MUST be corrected
  // against real hardware / vendor documentation.
  //////////////////////////////////////////////////////////////////////////

  // LCD (1.5" 240x240 IPS, ST7789-family controller)
  static constexpr size_t lcd_width_ = 240;
  static constexpr size_t lcd_height_ = 240;
  static constexpr size_t lcd_bytes_per_pixel = 2;
  static constexpr size_t frame_buffer_size = (((lcd_width_)*lcd_bytes_per_pixel) * lcd_height_);
  static constexpr int lcd_clock_speed = 80 * 1000 * 1000;
  static constexpr auto lcd_spi_num = SPI2_HOST;
  static constexpr gpio_num_t lcd_cs_io = GPIO_NUM_10;    // TODO: unverified
  static constexpr gpio_num_t lcd_mosi_io = GPIO_NUM_11;  // TODO: unverified
  static constexpr gpio_num_t lcd_sclk_io = GPIO_NUM_12;  // TODO: unverified
  static constexpr gpio_num_t lcd_dc_io = GPIO_NUM_13;    // TODO: unverified
  static constexpr gpio_num_t lcd_reset_io = GPIO_NUM_14; // TODO: unverified
  static constexpr gpio_num_t backlight_io = GPIO_NUM_15; // TODO: unverified
  static constexpr int lcd_offset_x = 0;
  static constexpr int lcd_offset_y = 0;
  static constexpr bool backlight_value = true;
  static constexpr bool reset_value = false;
  // 240x240 IPS ST7789 panels typically require color inversion
  static constexpr bool invert_colors = true; // TODO: unverified
  // the VMU Pro SDK exposes big-endian (byte-swapped) RGB565 colors
  static constexpr bool swap_color_order = true; // TODO: unverified
  static constexpr auto rotation = espp::DisplayRotation::LANDSCAPE;
  static constexpr bool swap_xy = false;
  static constexpr bool mirror_x = false;
  static constexpr bool mirror_y = false;

  // Buttons (all active low with internal pullups)
  static constexpr gpio_num_t dpad_up_io = GPIO_NUM_1;    // TODO: unverified
  static constexpr gpio_num_t dpad_down_io = GPIO_NUM_2;  // TODO: unverified
  static constexpr gpio_num_t dpad_left_io = GPIO_NUM_3;  // TODO: unverified
  static constexpr gpio_num_t dpad_right_io = GPIO_NUM_4; // TODO: unverified
  static constexpr gpio_num_t button_a_io = GPIO_NUM_5;   // TODO: unverified
  static constexpr gpio_num_t button_b_io = GPIO_NUM_6;   // TODO: unverified
  static constexpr gpio_num_t mode_io = GPIO_NUM_7;       // TODO: unverified
  static constexpr gpio_num_t power_io = GPIO_NUM_0;      // TODO: unverified
  static constexpr gpio_num_t bottom_io = GPIO_NUM_8;     // TODO: unverified

  // Audio (mono speaker driven by an I2S amplifier)
  static constexpr auto i2s_port = I2S_NUM_0;
  static constexpr gpio_num_t i2s_bck_io = GPIO_NUM_16;       // TODO: unverified
  static constexpr gpio_num_t i2s_ws_io = GPIO_NUM_17;        // TODO: unverified
  static constexpr gpio_num_t i2s_do_io = GPIO_NUM_18;        // TODO: unverified
  static constexpr gpio_num_t sound_enable_pin = GPIO_NUM_21; // TODO: unverified

  static constexpr int NUM_CHANNELS = 1;
  static constexpr int NUM_BYTES_PER_CHANNEL = 2;
  static constexpr int UPDATE_FREQUENCY = 60;

  static constexpr int calc_audio_buffer_size(int sample_rate) {
    return sample_rate * NUM_CHANNELS * NUM_BYTES_PER_CHANNEL / UPDATE_FREQUENCY;
  }

  // uSD card (SDMMC, 4-bit)
  static constexpr gpio_num_t sdcard_clk = GPIO_NUM_39; // TODO: unverified
  static constexpr gpio_num_t sdcard_cmd = GPIO_NUM_38; // TODO: unverified
  static constexpr gpio_num_t sdcard_d0 = GPIO_NUM_40;  // TODO: unverified
  static constexpr gpio_num_t sdcard_d1 = GPIO_NUM_41;  // TODO: unverified
  static constexpr gpio_num_t sdcard_d2 = GPIO_NUM_42;  // TODO: unverified
  static constexpr gpio_num_t sdcard_d3 = GPIO_NUM_47;  // TODO: unverified

  // sdcard
  sdmmc_card_t *sdcard_{nullptr};

  // Interrupts. One PinConfig per button, in the same order as the Button
  // enum so that button_state() can index by enum value.
  espp::Interrupt::PinConfig make_button_pin_config(gpio_num_t gpio, Button button) {
    return {.gpio_num = gpio,
            .callback =
                [this, button](const auto &event) {
                  if (button_callback_) {
                    button_callback_(button, event.active);
                  }
                },
            .active_level = espp::Interrupt::ActiveLevel::LOW,
            .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
            .pullup_enabled = true};
  }

  std::array<espp::Interrupt::PinConfig, NUM_BUTTONS> button_interrupt_pins_{
      make_button_pin_config(dpad_up_io, Button::DPAD_UP),
      make_button_pin_config(dpad_down_io, Button::DPAD_DOWN),
      make_button_pin_config(dpad_left_io, Button::DPAD_LEFT),
      make_button_pin_config(dpad_right_io, Button::DPAD_RIGHT),
      make_button_pin_config(button_a_io, Button::A),
      make_button_pin_config(button_b_io, Button::B),
      make_button_pin_config(mode_io, Button::MODE),
      make_button_pin_config(power_io, Button::POWER),
      make_button_pin_config(bottom_io, Button::BOTTOM),
  };

  // we'll only add the interrupt pins if the initialize method is called
  espp::Interrupt interrupts_{
      {.interrupts = {},
       .task_config = {.name = "vmu-pro interrupts",
                       .stack_size_bytes = CONFIG_VMU_PRO_INTERRUPT_STACK_SIZE,
                       .priority = CONFIG_VMU_PRO_INTERRUPT_PRIORITY,
                       .core_id = CONFIG_VMU_PRO_INTERRUPT_CORE_ID}}};

  // buttons
  std::atomic<bool> buttons_initialized_{false};
  button_callback_t button_callback_{nullptr};

  // display
  std::shared_ptr<Display<Pixel>> display_;
  std::shared_ptr<DisplayDriver> display_driver_{static_cast<DisplayDriver *>(nullptr)};
  std::vector<Led::ChannelConfig> backlight_channel_configs_{};
  std::shared_ptr<Led> backlight_{};
  static constexpr int spi_queue_size = 6;
  std::unique_ptr<Spi> lcd_spi_;
  std::unique_ptr<SpiPanelIo> lcd_;

  // sound
  std::atomic<bool> sound_initialized_{false};
  std::atomic<bool> mute_{false};
  std::atomic<float> volume_{50.0f};
  std::unique_ptr<espp::Task> audio_task_{nullptr};
  // i2s / low-level audio
  i2s_chan_handle_t audio_tx_handle{nullptr};
  std::vector<uint8_t> audio_tx_buffer;
  StreamBufferHandle_t audio_tx_stream{nullptr};
  i2s_std_config_t audio_std_cfg;
}; // class VmuPro
} // namespace espp
