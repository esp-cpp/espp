#include "smartpanlee-sc01-plus.hpp"

#include "esp_idf_version.h"
#ifndef ESP_IDF_VERSION_VAL
#define ESP_IDF_VERSION_VAL(major, minor, patch) (((major) << 16) | ((minor) << 8) | (patch))
#endif
#ifndef ESP_IDF_VERSION
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(0, 0, 0)
#endif

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>

#include <driver/i2s_std.h>
#include <driver/sdspi_host.h>
#include <esp_check.h>
#include <esp_heap_caps.h>
#include <esp_lcd_panel_ops.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>

namespace espp {

SmartPanleeSc01Plus::SmartPanleeSc01Plus()
    : BaseComponent("SmartPanleeSc01Plus") {}

bool SmartPanleeSc01Plus::initialize_touch(const SmartPanleeSc01Plus::touch_callback_t &callback) {
  if (touch_driver_) {
    logger_.warn("Touch already initialized, not initializing again!");
    return false;
  }

  std::error_code ec;
  touch_i2c_device_ = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = TouchDriver::DEFAULT_ADDRESS,
          .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!touch_i2c_device_) {
    logger_.error("Could not initialize touch I2C device: {}", ec.message());
    return false;
  }
  touch_driver_ = std::make_shared<TouchDriver>(TouchDriver::Config{
      .write = espp::make_i2c_addressed_write(touch_i2c_device_),
      .read_register = espp::make_i2c_addressed_read_register(touch_i2c_device_),
      .log_level = espp::Logger::Verbosity::WARN});

  touch_callback_ = callback;
  interrupts_.add_interrupt(touch_interrupt_pin_);
  update_touch();
  return true;
}

SmartPanleeSc01Plus::TouchpadData SmartPanleeSc01Plus::touchpad_data() const {
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  return touchpad_data_;
}

void SmartPanleeSc01Plus::touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y,
                                        uint8_t *btn_state) {
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  *num_touch_points = touchpad_data_.num_touch_points;
  *x = touchpad_data_.x;
  *y = touchpad_data_.y;
  *btn_state = touchpad_data_.btn_state;
}

SmartPanleeSc01Plus::TouchpadData
SmartPanleeSc01Plus::touchpad_convert(const SmartPanleeSc01Plus::TouchpadData &data) const {
  TouchpadData temp_data;
  temp_data.num_touch_points = data.num_touch_points;
  temp_data.x = data.x;
  temp_data.y = data.y;
  temp_data.btn_state = data.btn_state;
  if (temp_data.num_touch_points == 0) {
    return temp_data;
  }
  if (touch_swap_xy) {
    std::swap(temp_data.x, temp_data.y);
  }
  if (touch_invert_x) {
    temp_data.x = lcd_width_ - (temp_data.x + 1);
  }
  if (touch_invert_y) {
    temp_data.y = lcd_height_ - (temp_data.y + 1);
  }
  auto *display = lv_display_get_default();
  auto rotation = display ? lv_display_get_rotation(display) : LV_DISPLAY_ROTATION_0;
  switch (rotation) {
  case LV_DISPLAY_ROTATION_0:
    break;
  case LV_DISPLAY_ROTATION_90:
    temp_data.y = lcd_height_ - (temp_data.y + 1);
    std::swap(temp_data.x, temp_data.y);
    break;
  case LV_DISPLAY_ROTATION_180:
    temp_data.x = lcd_width_ - (temp_data.x + 1);
    temp_data.y = lcd_height_ - (temp_data.y + 1);
    break;
  case LV_DISPLAY_ROTATION_270:
    temp_data.x = lcd_width_ - (temp_data.x + 1);
    std::swap(temp_data.x, temp_data.y);
    break;
  default:
    break;
  }
  return temp_data;
}

bool SmartPanleeSc01Plus::panel_io_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                                                    esp_lcd_panel_io_event_data_t *edata,
                                                    void *user_ctx) {
  (void)panel_io;
  (void)edata;
  (void)user_ctx;
  auto *display = lv_display_get_default();
  if (display) {
    lv_display_flush_ready(display);
  }
  return false;
}

espp::Task::BaseConfig SmartPanleeSc01Plus::default_audio_task_config() {
  return {
      .name = "sc01+ audio",
      .stack_size_bytes = CONFIG_SMARTPANLEE_SC01_PLUS_AUDIO_TASK_STACK_SIZE,
      .priority = 20,
      .core_id = 1,
  };
}

bool SmartPanleeSc01Plus::initialize_i2s(uint32_t default_audio_rate) {
  logger_.info("Initializing I2S audio output at {} Hz", default_audio_rate);

  i2s_chan_config_t chan_config = I2S_CHANNEL_DEFAULT_CONFIG(i2s_port, I2S_ROLE_MASTER);
  chan_config.auto_clear = true;
  ESP_ERROR_CHECK(i2s_new_channel(&chan_config, &audio_tx_handle_, nullptr));

  auto pins = i2s_pins();
  audio_std_cfg_ = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(default_audio_rate),
      .slot_cfg =
          I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg =
          {
              .mclk = pins.mclk,
              .bclk = pins.bclk,
              .ws = pins.ws,
              .dout = pins.dout,
              .din = pins.din,
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };
  audio_std_cfg_.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_tx_handle_, &audio_std_cfg_));

  auto buffer_size = calc_audio_buffer_size(default_audio_rate);
  audio_tx_buffer_.resize(buffer_size);
  audio_tx_stream_ = xStreamBufferCreate(buffer_size * 4, 0);
  if (!audio_tx_stream_) {
    logger_.error("Failed to create audio stream buffer");
    return false;
  }
  xStreamBufferReset(audio_tx_stream_);

  ESP_ERROR_CHECK(i2s_channel_enable(audio_tx_handle_));
  return true;
}

bool SmartPanleeSc01Plus::initialize_lcd() {
  if (panel_io_ || backlight_) {
    logger_.warn("LCD already initialized, not initializing again!");
    return false;
  }

  esp_lcd_i80_bus_config_t bus_config = {
    .dc_gpio_num = lcd_dc_io,
    .wr_gpio_num = lcd_wr_io,
    .clk_src = LCD_CLK_SRC_DEFAULT,
    .data_gpio_nums = {lcd_d0_io, lcd_d1_io, lcd_d2_io, lcd_d3_io, lcd_d4_io, lcd_d5_io, lcd_d6_io,
                       lcd_d7_io},
    .bus_width = 8,
    .max_transfer_bytes = lcd_max_transfer_bytes,
    .dma_burst_size = 64,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    .flags = {},
#endif
  };
  ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &lcd_bus_));

  esp_lcd_panel_io_i80_config_t io_config = {
    .cs_gpio_num = lcd_cs_io,
    .pclk_hz = lcd_clock_speed_hz,
    .trans_queue_depth = 10,
    .on_color_trans_done = nullptr,
    .user_ctx = nullptr,
    .lcd_cmd_bits = 8,
    .lcd_param_bits = 8,
    .dc_levels =
        {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    .flags = {},
#endif
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(lcd_bus_, &io_config, &panel_io_));

  using namespace std::placeholders;
  display_driver_ = std::make_shared<DisplayDriver>(espp::display_drivers::Config{
      .write_command = std::bind(&SmartPanleeSc01Plus::write_command, this, _1, _2, _3),
      .lcd_send_lines =
          std::bind(&SmartPanleeSc01Plus::write_lcd_lines, this, _1, _2, _3, _4, _5, _6),
      .reset_pin = lcd_reset_io,
      .data_command_pin = GPIO_NUM_NC,
      .reset_value = false,
      .invert_colors = invert_colors,
      .swap_color_order = swap_color_order,
      .swap_xy = swap_xy,
      .mirror_x = mirror_x,
      .mirror_y = mirror_y});
  if (!display_driver_ || !display_driver_->initialize()) {
    display_driver_.reset();
    return false;
  }

  backlight_channel_configs_.push_back({.gpio = static_cast<size_t>(lcd_backlight_io),
                                        .channel = LEDC_CHANNEL_0,
                                        .timer = LEDC_TIMER_0,
                                        .output_invert = !backlight_value});
  backlight_ = std::make_shared<Led>(Led::Config{.timer = LEDC_TIMER_0,
                                                 .frequency_hz = 5000,
                                                 .channels = backlight_channel_configs_,
                                                 .duty_resolution = LEDC_TIMER_10_BIT});

  brightness(100.0f);
  return true;
}

bool SmartPanleeSc01Plus::initialize_display(size_t pixel_buffer_size) {
  if (!panel_io_ || !display_driver_) {
    logger_.error(
        "LCD not initialized, you must call initialize_lcd() before initialize_display()!");
    return false;
  }
  if (display_) {
    logger_.warn("Display already initialized, not initializing again!");
    return false;
  }

  display_ = std::make_shared<Display<Pixel>>(
      Display<Pixel>::LvglConfig{
          .width = lcd_width_,
          .height = lcd_height_,
          .flush_callback =
              [this](lv_display_t *disp, const lv_area_t *area, uint8_t *color_map) {
                if (display_driver_) {
                  display_driver_->flush(disp, area, color_map);
                }
              },
          .rotation_callback =
              [this](const DisplayRotation &new_rotation) {
                if (display_driver_) {
                  display_driver_->set_rotation(new_rotation);
                }
              },
          .rotation = rotation},
      Display<Pixel>::OledConfig{
          .set_brightness_callback =
              [this](float brightness) { this->brightness(brightness * 100); },
          .get_brightness_callback = [this]() { return this->brightness() / 100.0f; }},
      Display<Pixel>::DynamicMemoryConfig{
          .pixel_buffer_size = pixel_buffer_size,
          .double_buffered = true,
          .allocation_flags = MALLOC_CAP_8BIT | MALLOC_CAP_DMA,
      });

  touchpad_input_ = std::make_shared<TouchpadInput>(TouchpadInput::Config{
      .touchpad_read =
          std::bind(&SmartPanleeSc01Plus::touchpad_read, this, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
      .swap_xy = touch_swap_xy,
      .invert_x = touch_invert_x,
      .invert_y = touch_invert_y,
      .log_level = espp::Logger::Verbosity::WARN});

  const esp_lcd_panel_io_callbacks_t callbacks = {
      .on_color_trans_done = &SmartPanleeSc01Plus::panel_io_color_trans_done,
  };
  ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(panel_io_, &callbacks, nullptr));

  return true;
}

bool SmartPanleeSc01Plus::initialize_audio() {
  return initialize_audio(44100, default_audio_task_config());
}

bool SmartPanleeSc01Plus::initialize_audio(uint32_t sample_rate) {
  return initialize_audio(sample_rate, default_audio_task_config());
}

bool SmartPanleeSc01Plus::initialize_audio(uint32_t sample_rate,
                                           const espp::Task::BaseConfig &task_config) {
  if (audio_initialized_) {
    logger_.warn("Audio already initialized");
    return true;
  }
  if (!initialize_i2s(sample_rate)) {
    logger_.error("Could not initialize I2S audio output");
    return false;
  }

  using namespace std::placeholders;
  audio_task_ = espp::Task::make_unique({
      .callback = std::bind(&SmartPanleeSc01Plus::audio_task_callback, this, _1, _2, _3),
      .task_config = task_config,
  });

  audio_initialized_ = audio_task_ && audio_task_->start();
  return audio_initialized_;
}

void SmartPanleeSc01Plus::brightness(float brightness) {
  if (!backlight_) {
    return;
  }
  backlight_->set_duty(backlight_channel_configs_.front().channel,
                       std::clamp(brightness, 0.0f, 100.0f));
}

float SmartPanleeSc01Plus::brightness() const {
  if (!backlight_) {
    return 0.0f;
  }
  auto duty = backlight_->get_duty(backlight_channel_configs_.front().channel);
  return duty.value_or(0.0f);
}

void SmartPanleeSc01Plus::audio_sample_rate(uint32_t sample_rate) {
  if (!audio_initialized_ || !audio_tx_handle_) {
    return;
  }

  logger_.info("Setting audio sample rate to {} Hz", sample_rate);
  i2s_channel_disable(audio_tx_handle_);
  audio_std_cfg_.clk_cfg.sample_rate_hz = sample_rate;
  i2s_channel_reconfig_std_clock(audio_tx_handle_, &audio_std_cfg_.clk_cfg);
  xStreamBufferReset(audio_tx_stream_);
  i2s_channel_enable(audio_tx_handle_);
}

uint32_t SmartPanleeSc01Plus::audio_sample_rate() const {
  if (!audio_initialized_) {
    return 0;
  }
  return audio_std_cfg_.clk_cfg.sample_rate_hz;
}

size_t SmartPanleeSc01Plus::audio_buffer_size() const { return audio_tx_buffer_.size(); }

void SmartPanleeSc01Plus::mute(bool mute) { mute_ = mute; }

bool SmartPanleeSc01Plus::is_muted() const { return mute_; }

void SmartPanleeSc01Plus::volume(float volume) { volume_ = std::clamp(volume, 0.0f, 100.0f); }

float SmartPanleeSc01Plus::volume() const { return volume_; }

void SmartPanleeSc01Plus::play_audio(std::span<const uint8_t> data) {
  play_audio(data.data(), static_cast<uint32_t>(data.size()));
}

void SmartPanleeSc01Plus::play_audio(const uint8_t *data, uint32_t num_bytes) {
  if (!audio_initialized_ || !audio_tx_stream_ || !data || num_bytes == 0) {
    return;
  }
  xStreamBufferSend(audio_tx_stream_, data, num_bytes, 0);
}

size_t SmartPanleeSc01Plus::rotated_display_width() const {
  auto *display = lv_display_get_default();
  auto rotation = display ? lv_display_get_rotation(display) : LV_DISPLAY_ROTATION_0;
  switch (rotation) {
  case LV_DISPLAY_ROTATION_90:
  case LV_DISPLAY_ROTATION_270:
    return lcd_height_;
  case LV_DISPLAY_ROTATION_0:
  case LV_DISPLAY_ROTATION_180:
  default:
    return lcd_width_;
  }
}

size_t SmartPanleeSc01Plus::rotated_display_height() const {
  auto *display = lv_display_get_default();
  auto rotation = display ? lv_display_get_rotation(display) : LV_DISPLAY_ROTATION_0;
  switch (rotation) {
  case LV_DISPLAY_ROTATION_90:
  case LV_DISPLAY_ROTATION_270:
    return lcd_width_;
  case LV_DISPLAY_ROTATION_0:
  case LV_DISPLAY_ROTATION_180:
  default:
    return lcd_height_;
  }
}

bool SmartPanleeSc01Plus::initialize_sdcard(const SmartPanleeSc01Plus::SdCardConfig &config) {
  if (sdcard_) {
    logger_.error("SD card already initialized!");
    return false;
  }

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
  mount_config.format_if_mount_failed = config.format_if_mount_failed;
  mount_config.max_files = config.max_files;
  mount_config.allocation_unit_size = config.allocation_unit_size;

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = SPI2_HOST;
  host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

  spi_bus_config_t bus_config = {};
  bus_config.mosi_io_num = sd_card_pins().mosi;
  bus_config.miso_io_num = sd_card_pins().miso;
  bus_config.sclk_io_num = sd_card_pins().clk;
  bus_config.quadwp_io_num = GPIO_NUM_NC;
  bus_config.quadhd_io_num = GPIO_NUM_NC;
  bus_config.max_transfer_sz = 4 * 1024;
  auto ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_config, SDSPI_DEFAULT_DMA);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    logger_.error("Failed to initialize SD SPI bus ({})", esp_err_to_name(ret));
    return false;
  }

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = sd_card_pins().cs;
  slot_config.host_id = (spi_host_device_t)host.slot;

  ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &sdcard_);
  if (ret != ESP_OK) {
    logger_.error("Failed to initialize the card ({})", esp_err_to_name(ret));
    return false;
  }

  sd_card_initialized_ = true;
  return true;
}

bool SmartPanleeSc01Plus::initialize_sdcard() { return initialize_sdcard(SdCardConfig{}); }

bool SmartPanleeSc01Plus::is_sd_card_available() const { return sd_card_initialized_; }

bool SmartPanleeSc01Plus::get_sd_card_info(uint32_t *size_mb, uint32_t *free_mb) const {
  if (!sd_card_initialized_) {
    return false;
  }

  uint64_t total_bytes = 0;
  uint64_t free_bytes = 0;
  auto ret = esp_vfs_fat_info(mount_point, &total_bytes, &free_bytes);
  if (ret != ESP_OK) {
    logger_.error("Failed to get SD card information ({})", esp_err_to_name(ret));
    return false;
  }

  if (size_mb) {
    *size_mb = total_bytes / (1024 * 1024);
  }
  if (free_mb) {
    *free_mb = free_bytes / (1024 * 1024);
  }
  return true;
}

bool SmartPanleeSc01Plus::update_touch() {
  if (!touch_driver_) {
    return false;
  }

  std::error_code ec;
  TouchpadData temp_data;
  touch_driver_->get_touch_point(&temp_data.num_touch_points, &temp_data.x, &temp_data.y, ec);
  if (ec) {
    logger_.error("Could not update touch driver: {}", ec.message());
    std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
    touchpad_data_ = {};
    return false;
  }

  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  bool changed = temp_data != touchpad_data_;
  touchpad_data_ = temp_data;
  return changed;
}

bool SmartPanleeSc01Plus::audio_task_callback(std::mutex &m, std::condition_variable &cv,
                                              bool &task_notified) {
  (void)m;
  (void)cv;
  (void)task_notified;

  auto available = xStreamBufferBytesAvailable(audio_tx_stream_);
  auto buffer_size = audio_tx_buffer_.size();
  available = std::min(available, buffer_size);

  uint8_t *buffer = audio_tx_buffer_.data();
  std::memset(buffer, 0, buffer_size);

  size_t bytes_received = 0;
  if (available > 0) {
    bytes_received = xStreamBufferReceive(audio_tx_stream_, buffer, available, 0);
  }

  if (mute_) {
    std::memset(buffer, 0, bytes_received);
  } else {
    auto volume = volume_.load();
    if (volume < 100.0f) {
      auto *samples = reinterpret_cast<int16_t *>(buffer);
      auto num_samples = bytes_received / sizeof(int16_t);
      for (size_t i = 0; i < num_samples; ++i) {
        auto scaled = static_cast<int32_t>((samples[i] * volume) / 100.0f);
        samples[i] = static_cast<int16_t>(std::clamp<int32_t>(
            scaled, std::numeric_limits<int16_t>::min(), std::numeric_limits<int16_t>::max()));
      }
    }
  }

  i2s_channel_write(audio_tx_handle_, buffer, buffer_size, nullptr, portMAX_DELAY);
  return false;
}

void SmartPanleeSc01Plus::write_command(uint8_t command, std::span<const uint8_t> parameters,
                                        uint32_t user_data) {
  (void)user_data;
  const uint8_t *data = parameters.empty() ? nullptr : parameters.data();
  ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(panel_io_, command, data, parameters.size()));
}

void SmartPanleeSc01Plus::write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data,
                                          uint32_t user_data) {
  (void)user_data;
  std::array<uint8_t, 4> window = {
      static_cast<uint8_t>((xs >> 8) & 0xFF),
      static_cast<uint8_t>(xs & 0xFF),
      static_cast<uint8_t>((xe >> 8) & 0xFF),
      static_cast<uint8_t>(xe & 0xFF),
  };
  ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(panel_io_,
                                            static_cast<uint8_t>(DisplayDriver::Command::caset),
                                            window.data(), window.size()));
  window = {
      static_cast<uint8_t>((ys >> 8) & 0xFF),
      static_cast<uint8_t>(ys & 0xFF),
      static_cast<uint8_t>((ye >> 8) & 0xFF),
      static_cast<uint8_t>(ye & 0xFF),
  };
  ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(panel_io_,
                                            static_cast<uint8_t>(DisplayDriver::Command::raset),
                                            window.data(), window.size()));
  size_t num_bytes =
      static_cast<size_t>(xe - xs + 1) * static_cast<size_t>(ye - ys + 1) * sizeof(Pixel);
  ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(
      panel_io_, static_cast<int>(DisplayDriver::Command::ramwr), data, num_bytes));
}

} // namespace espp
