#pragma once

#include <memory>
#include <string>
#include <vector>

#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <driver/spi_master.h>
#include <hal/spi_types.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>

#include "base_component.hpp"
#include "es7210.hpp"
#include "es8311.hpp"
#include "gt911.hpp"
#include "i2c.hpp"
#include "interrupt.hpp"
#include "st7789.hpp"
#include "touchpad_input.hpp"
#include "tt21100.hpp"

namespace espp {
/// The EspBox class provides an interface to the ESP32-S3-BOX and
/// ESP32-S3-BOX-3 development boards.
///
/// The class provides access to the following features:
/// - Touchpad
/// - Display
/// - Audio
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \section esp_box_example Example
/// \snippet esp_box_example.cpp esp box example
class EspBox : public BaseComponent {
public:
  /// Alias for the pixel type used by the ESP-Box display
  using Pixel = lv_color16_t;

  /// Alias for the display driver used by the ESP-Box display
  using DisplayDriver = espp::St7789;
  using TouchpadData = espp::TouchpadData;

  /// The type of the box
  enum class BoxType {
    UNKNOWN, ///< unknown box
    BOX,     ///< ESP32-S3-BOX
    BOX3,    ///< ESP32-S3-BOX-3
  };

  using touch_callback_t = std::function<void(const TouchpadData &)>;

  /// @brief Access the singleton instance of the EspBox class
  /// @return Reference to the singleton instance of the EspBox class
  static EspBox &get() {
    static EspBox instance;
    return instance;
  }

  EspBox(const EspBox &) = delete;
  EspBox &operator=(const EspBox &) = delete;
  EspBox(EspBox &&) = delete;
  EspBox &operator=(EspBox &&) = delete;

  /// Get the type of the box
  /// \return The type of the box that was detected
  /// \see BoxType
  BoxType box_type() const;

  /// Get a reference to the internal I2C bus
  /// \return A reference to the internal I2C bus
  /// \note The internal I2C bus is used for the touchscreen and audio codec
  I2c &internal_i2c();

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

  /////////////////////////////////////////////////////////////////////////////
  // Audio
  /////////////////////////////////////////////////////////////////////////////

  /// Get the GPIO pin for the mute button (top of the box)
  /// \return The GPIO pin for the mute button
  static constexpr auto get_mute_pin() { return mute_pin; }

  /// Initialize the sound subsystem
  /// \param default_audio_rate The default audio rate
  /// \return true if the sound subsystem was successfully initialized, false
  ///         otherwise
  bool initialize_sound(uint32_t default_audio_rate = 48000);

  /// Enable or disable sound
  /// \note This method sets the power pin to the appropriate value
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
  /// \param mute true to mute the audio, false to unmute the audio
  void mute(bool mute);

  /// Check if the audio is muted
  /// \return true if the audio is muted, false otherwise
  bool is_muted() const;

  /// Set the volume
  /// \param volume The volume in percent (0 - 100)
  void volume(float volume);

  /// Get the volume
  /// \return The volume in percent (0 - 100)
  float volume() const;

  /// Play audio
  /// \param data The audio data to play
  void play_audio(const std::vector<uint8_t> &data);

  /// Play audio
  /// \param data The audio data to play
  /// \param num_bytes The number of bytes to play
  void play_audio(const uint8_t *data, uint32_t num_bytes);

protected:
  EspBox();
  void detect();
  bool initialize_codec();
  bool initialize_i2s(uint32_t default_audio_rate);
  bool update_touch();
  bool update_gt911();
  bool update_tt21100();
  void update_volume_output();
  bool audio_task_callback(std::mutex &m, std::condition_variable &cv);
  void lcd_wait_lines();

  // box 3:
  struct box3 {
    static constexpr gpio_num_t backlight_io = GPIO_NUM_47; // was 45 on ESP32-S3-BOX
    static constexpr bool reset_value = true;               // was false on ESP32-S3-BOX
    static constexpr gpio_num_t i2s_ws_io = GPIO_NUM_45;    // was 47 on ESP32-S3-BOX
    static constexpr bool touch_invert_x = false;           // was true on ESP32-S3-BOX
    static constexpr auto touch_interrupt_level = espp::Interrupt::ActiveLevel::HIGH;
    static constexpr auto touch_interrupt_type = espp::Interrupt::Type::RISING_EDGE;
    static constexpr auto touch_interrupt_pullup_enabled = false;
  }; // struct box3

  // box:
  struct box {
    static constexpr gpio_num_t backlight_io = GPIO_NUM_45;
    static constexpr bool reset_value = false;
    static constexpr gpio_num_t i2s_ws_io = GPIO_NUM_47;
    static constexpr bool touch_invert_x = true;
    static constexpr auto touch_interrupt_level = espp::Interrupt::ActiveLevel::LOW;
    static constexpr auto touch_interrupt_type = espp::Interrupt::Type::FALLING_EDGE;
    static constexpr auto touch_interrupt_pullup_enabled = true;
  }; // struct box

  // set by the detect() method using the box3 and box namespaces
  gpio_num_t backlight_io;
  bool reset_value;
  gpio_num_t i2s_ws_io;
  bool touch_invert_x;
  espp::Interrupt::ActiveLevel touch_interrupt_level = box::touch_interrupt_level;
  espp::Interrupt::Type touch_interrupt_type;
  bool touch_interrupt_pullup_enabled;

  // common:
  // internal i2c (touchscreen, audio codec)
  static constexpr auto internal_i2c_port = I2C_NUM_0;
  static constexpr auto internal_i2c_clock_speed = 400 * 1000;
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_8;
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_18;

  // LCD
  static constexpr size_t lcd_width_ = 320;
  static constexpr size_t lcd_height_ = 240;
  static constexpr size_t lcd_bytes_per_pixel = 2;
  static constexpr size_t frame_buffer_size = (((lcd_width_)*lcd_bytes_per_pixel) * lcd_height_);
  static constexpr int lcd_clock_speed = 60 * 1000 * 1000;
  static constexpr auto lcd_spi_num = SPI2_HOST;
  static constexpr gpio_num_t lcd_cs_io = GPIO_NUM_5;
  static constexpr gpio_num_t lcd_mosi_io = GPIO_NUM_6;
  static constexpr gpio_num_t lcd_sclk_io = GPIO_NUM_7;
  static constexpr gpio_num_t lcd_reset_io = GPIO_NUM_48;
  static constexpr gpio_num_t lcd_dc_io = GPIO_NUM_4;
  static constexpr bool backlight_value = true;
  static constexpr bool invert_colors = true;
  static constexpr auto rotation = espp::DisplayRotation::LANDSCAPE;
  static constexpr bool mirror_x = true;
  static constexpr bool mirror_y = true;
  static constexpr bool swap_xy = false;
  static constexpr bool swap_color_order = true;

  // touch
  static constexpr bool touch_swap_xy = false;
  static constexpr bool touch_invert_y = false;
  static constexpr gpio_num_t touch_interrupt = GPIO_NUM_3;

  // sound
  static constexpr gpio_num_t sound_power_pin = GPIO_NUM_46;
  static constexpr auto i2s_port = I2S_NUM_0;
  static constexpr gpio_num_t i2s_mck_io = GPIO_NUM_2;
  static constexpr gpio_num_t i2s_bck_io = GPIO_NUM_17;
  static constexpr gpio_num_t i2s_do_io = GPIO_NUM_15;
  static constexpr gpio_num_t i2s_di_io = GPIO_NUM_16;
  static constexpr gpio_num_t mute_pin = GPIO_NUM_1;

  static constexpr int NUM_CHANNELS = 2;
  static constexpr int NUM_BYTES_PER_CHANNEL = 2;
  static constexpr int UPDATE_FREQUENCY = 60;

  static constexpr int calc_audio_buffer_size(int sample_rate) {
    return sample_rate * NUM_CHANNELS * NUM_BYTES_PER_CHANNEL / UPDATE_FREQUENCY;
  }

  BoxType box_type_{BoxType::UNKNOWN};

  // TODO: allow core id configuration
  I2c internal_i2c_{{.port = internal_i2c_port,
                     .sda_io_num = internal_i2c_sda,
                     .scl_io_num = internal_i2c_scl,
                     .sda_pullup_en = GPIO_PULLUP_ENABLE,
                     .scl_pullup_en = GPIO_PULLUP_ENABLE}};

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
  };

  // we'll only add each interrupt pin if the initialize method is called
  espp::Interrupt interrupts_{
      {.interrupts = {},
       .task_config = {.name = "esp-box interrupts",
                       .stack_size_bytes = CONFIG_ESP_BOX_INTERRUPT_STACK_SIZE}}};

  // touch
  std::shared_ptr<Gt911> gt911_;     // only used on ESP32-S3-BOX-3
  std::shared_ptr<Tt21100> tt21100_; // only used on ESP32-S3-BOX
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

  // sound
  std::atomic<float> volume_{50.0f};
  std::atomic<bool> mute_{false};
  std::unique_ptr<espp::Task> audio_task_{nullptr};
  // i2s / low-level audio
  i2s_chan_handle_t audio_tx_handle{nullptr};
  std::vector<uint8_t> audio_tx_buffer;
  StreamBufferHandle_t audio_tx_stream;
  i2s_std_config_t audio_std_cfg;
  i2s_event_callbacks_t audio_tx_callbacks_;
  std::atomic<bool> has_sound{false};
}; // class EspBox
} // namespace espp

// for easy printing of BoxType using libfmt
template <> struct fmt::formatter<espp::EspBox::BoxType> : fmt::formatter<std::string> {
  template <typename FormatContext> auto format(espp::EspBox::BoxType c, FormatContext &ctx) const {
    std::string name;
    switch (c) {
    case espp::EspBox::BoxType::UNKNOWN:
      name = "UNKNOWN";
      break;
    case espp::EspBox::BoxType::BOX:
      name = "BOX";
      break;
    case espp::EspBox::BoxType::BOX3:
      name = "BOX3";
      break;
    }
    return formatter<std::string>::format(name, ctx);
  }
};
