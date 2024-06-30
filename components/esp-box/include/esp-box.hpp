#pragma once

#include <memory>
#include <string>
#include <vector>

#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <driver/spi_master.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>

#include "base_component.hpp"
#include "gt911.hpp"
#include "i2c.hpp"
#include "st7789.hpp"
#include "touchpad_input.hpp"
#include "tt21100.hpp"

namespace espp {
class EspBox : public BaseComponent {
public:
  /// The type of the box
  enum class BoxType {
    UNKNOWN, ///< unknown box
    BOX,     ///< ESP32-S3-BOX
    BOX3,    ///< ESP32-S3-BOX-3
  };

  /// The data structure for the touchpad
  struct TouchpadData {
    uint8_t num_touch_points = 0; ///< The number of touch points
    uint16_t x = 0;               ///< The x coordinate
    uint16_t y = 0;               ///< The y coordinate
    uint8_t btn_state = 0;        ///< The button state (0 = button released, 1 = button pressed)
  };

  struct Config {};

  EspBox();

  BoxType box_type() const;

  I2c &internal_i2c();

  // Touch
  bool initialize_touch();
  bool update_touch();
  std::shared_ptr<TouchpadInput> touchpad_input() const;
  TouchpadData touchpad_data() const;
  void touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y, uint8_t *btn_state);

  // Display
  bool initialize_lcd();
  bool initialize_display(size_t pixel_buffer_size);
  static constexpr size_t lcd_width() { return lcd_width_; }
  static constexpr size_t lcd_height() { return lcd_height_; }
  std::shared_ptr<Display> display() const;
  uint16_t *vram0() const;
  uint16_t *vram1() const;
  uint8_t *frame_buffer0() const;
  uint8_t *frame_buffer1() const;
  void write_lcd(const uint8_t *data, size_t length, uint32_t user_data);
  void write_lcd_frame(const uint16_t x, const uint16_t y, const uint16_t width,
                       const uint16_t height, const uint8_t *data);
  void write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data, uint32_t user_data);
  void brightness(float brightness);
  float brightness() const;

  // Audio
  bool initialize_sound(uint32_t default_audio_rate = 48000);
  uint32_t audio_sample_rate() const;
  void audio_sample_rate(uint32_t sample_rate);
  void mute(bool mute);
  bool is_muted() const;
  void volume(float volume);
  float volume() const;
  void play_audio(const std::vector<uint8_t> &data);
  void play_audio(const uint8_t *data, uint32_t num_bytes);

  static constexpr auto get_lcd_dc_gpio() { return lcd_dc_io; }

protected:
  void detect();
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
  };                                                        // struct box3

  // box:
  struct box {
    static constexpr gpio_num_t backlight_io = GPIO_NUM_45;
    static constexpr bool reset_value = false;
    static constexpr gpio_num_t i2s_ws_io = GPIO_NUM_47;
  }; // struct box

  // set by the detect() method using the box3 and box namespaces
  gpio_num_t backlight_io;
  bool reset_value;
  gpio_num_t i2s_ws_io;

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
  static constexpr auto rotation = espp::Display::Rotation::LANDSCAPE;
  static constexpr bool mirror_x = true;
  static constexpr bool mirror_y = true;
  using DisplayDriver = espp::St7789;

  // touch
  static constexpr bool touch_swap_xy = false;
  static constexpr bool touch_invert_x = true;
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

  // touch
  std::shared_ptr<Gt911> gt911_;     // only used on ESP32-S3-BOX-3
  std::shared_ptr<Tt21100> tt21100_; // only used on ESP32-S3-BOX
  std::shared_ptr<TouchpadInput> touchpad_input_;
  std::recursive_mutex touchpad_data_mutex_;
  TouchpadData touchpad_data_;

  // display
  std::shared_ptr<Display> display_;
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
  std::atomic<float> volume_{1.0f};
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
  template <typename FormatContext> auto format(espp::EspBox::BoxType c, FormatContext &ctx) {
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

// for easy printing of TouchpadData using libfmt
template <> struct fmt::formatter<espp::EspBox::TouchpadData> : fmt::formatter<std::string> {
  template <typename FormatContext>
  auto format(const espp::EspBox::TouchpadData &c, FormatContext &ctx) {
    return fmt::format_to(ctx.out(),
                          "TouchpadData{{num_touch_points={}, x={}, y={}, btn_state={}}}",
                          c.num_touch_points, c.x, c.y, c.btn_state);
  }
};
