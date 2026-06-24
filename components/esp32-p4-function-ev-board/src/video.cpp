#include "esp32-p4-function-ev-board.hpp"

#include "esp_idf_version.h"
#ifndef ESP_IDF_VERSION_VAL
#define ESP_IDF_VERSION_VAL(major, minor, patch) (((major) << 16) | ((minor) << 8) | (patch))
#endif
#ifndef ESP_IDF_VERSION
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(0, 0, 0)
#endif

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>

#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_interface.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_ldo_regulator.h>

using namespace std::chrono_literals;

namespace espp {

bool Esp32P4FunctionEvBoard::initialize_lcd() {
  logger_.info("Initializing LCD (MIPI-DSI)");

  esp_err_t ret = ESP_OK;

  // Enable the MIPI DSI PHY power LDO (on-chip LDO_VO3 -> VDD_MIPI_DPHY)
  static esp_ldo_channel_handle_t phy_pwr_chan = nullptr;
  if (phy_pwr_chan == nullptr) {
    esp_ldo_channel_config_t phy_pwr_cfg{};
    phy_pwr_cfg.chan_id = mipi_dsi_phy_ldo_channel;
    phy_pwr_cfg.voltage_mv = mipi_dsi_phy_ldo_voltage_mv;
    ret = esp_ldo_acquire_channel(&phy_pwr_cfg, &phy_pwr_chan);
    if (ret != ESP_OK) {
      logger_.error("Failed to acquire MIPI DSI PHY power LDO channel: {}", esp_err_to_name(ret));
      return false;
    }
  }

  // Hardware reset on the EK79007 reset GPIO (active-low) before detection. This
  // is harmless if an ILI9881C is attached (its reset line is not connected on
  // this board; it is reset over DSI during init).
  {
    constexpr gpio_num_t pre_reset_io = GPIO_NUM_27;
    logger_.info("Performing LCD hardware reset on GPIO{}", static_cast<int>(pre_reset_io));
    gpio_config_t rst_cfg{};
    rst_cfg.pin_bit_mask = 1ULL << static_cast<int>(pre_reset_io);
    rst_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_config(&rst_cfg);
    gpio_set_level(pre_reset_io, 0); // assert reset
    std::this_thread::sleep_for(10ms);
    gpio_set_level(pre_reset_io, 1); // release reset
    std::this_thread::sleep_for(120ms);
  }

  // Create the MIPI DSI bus (also initializes the DSI PHY)
  if (lcd_handles_.mipi_dsi_bus == nullptr) {
    logger_.info("Creating MIPI DSI bus ({} lanes, {} Mbps/lane)", mipi_dsi_lanes,
                 mipi_dsi_lane_bitrate_mbps);
    esp_lcd_dsi_bus_config_t bus_config = {};
    bus_config.bus_id = 0;
    bus_config.num_data_lanes = mipi_dsi_lanes;
    bus_config.phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT;
    bus_config.lane_bit_rate_mbps = mipi_dsi_lane_bitrate_mbps;
    ret = esp_lcd_new_dsi_bus(&bus_config, &lcd_handles_.mipi_dsi_bus);
    if (ret != ESP_OK) {
      logger_.error("New DSI bus init failed: {}", esp_err_to_name(ret));
      return false;
    }
  }

  // Install the DBI panel IO (used to send DCS commands/parameters)
  if (lcd_handles_.io == nullptr) {
    logger_.info("Installing MIPI DSI DBI panel IO");
    esp_lcd_dbi_io_config_t dbi_config = {};
    dbi_config.virtual_channel = 0;
    dbi_config.lcd_cmd_bits = 8;
    dbi_config.lcd_param_bits = 8;
    ret = esp_lcd_new_panel_io_dbi(lcd_handles_.mipi_dsi_bus, &dbi_config, &lcd_handles_.io);
    if (ret != ESP_OK) {
      logger_.error("New panel IO failed: {}", esp_err_to_name(ret));
      return false;
    }
  }

  // Select the panel (Kconfig-driven) and apply its parameters (geometry, DPI
  // timing, backlight/reset GPIOs).
  apply_panel_params(default_controller_);
  logger_.info("Using display panel: {} ({}x{})", get_display_controller_name(), display_width_,
               display_height_);

  // Configure the backlight PWM on the detected panel's backlight GPIO
  if (!backlight_) {
    backlight_channel_configs_.push_back({.gpio = static_cast<size_t>(panel_params_.backlight_io),
                                          .channel = LEDC_CHANNEL_0,
                                          .timer = LEDC_TIMER_0,
                                          .duty = 0.0f,
                                          .speed_mode = LEDC_LOW_SPEED_MODE,
                                          .output_invert = !backlight_value});
    backlight_ = std::make_shared<Led>(Led::Config{.timer = LEDC_TIMER_0,
                                                   .frequency_hz = 5000,
                                                   .channels = backlight_channel_configs_,
                                                   .duty_resolution = LEDC_TIMER_10_BIT});
  }
  brightness(100.0f);

  // Create the DPI (video) panel with the detected panel's timing. This matches
  // Espressif's esp_lcd_ek79007/ili9881c flow: the DPI panel is created first,
  // then the vendor init commands are sent (command mode), and finally the DPI
  // panel is started (panel->init()).
  if (lcd_handles_.panel == nullptr) {
    esp_lcd_dpi_panel_config_t dpi_cfg{};
    memset(&dpi_cfg, 0, sizeof(dpi_cfg));
    dpi_cfg.virtual_channel = 0;
    dpi_cfg.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    dpi_cfg.dpi_clock_freq_mhz = panel_params_.dpi_clock_freq_mhz;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    dpi_cfg.in_color_format = LCD_COLOR_FMT_RGB565;
    dpi_cfg.out_color_format = LCD_COLOR_FMT_RGB565;
#else
    dpi_cfg.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
    dpi_cfg.flags.use_dma2d = true;
#endif
    dpi_cfg.num_fbs = 1;
    dpi_cfg.video_timing.h_size = display_width_;
    dpi_cfg.video_timing.v_size = display_height_;
    dpi_cfg.video_timing.hsync_pulse_width = panel_params_.hsync_pulse_width;
    dpi_cfg.video_timing.hsync_back_porch = panel_params_.hsync_back_porch;
    dpi_cfg.video_timing.hsync_front_porch = panel_params_.hsync_front_porch;
    dpi_cfg.video_timing.vsync_pulse_width = panel_params_.vsync_pulse_width;
    dpi_cfg.video_timing.vsync_back_porch = panel_params_.vsync_back_porch;
    dpi_cfg.video_timing.vsync_front_porch = panel_params_.vsync_front_porch;
    logger_.info("Creating DPI panel ({}x{} @ {} MHz)", dpi_cfg.video_timing.h_size,
                 dpi_cfg.video_timing.v_size, dpi_cfg.dpi_clock_freq_mhz);
    ret = esp_lcd_new_panel_dpi(lcd_handles_.mipi_dsi_bus, &dpi_cfg, &lcd_handles_.panel);
    if (ret != ESP_OK) {
      logger_.error("Failed to create MIPI DSI DPI panel: {}", esp_err_to_name(ret));
      return false;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    // On ESP-IDF >= 6.0 the DPI panel needs DMA2D explicitly enabled for
    // esp_lcd_panel_draw_bitmap() (the LVGL flush path) to copy into the
    // framebuffer; without it the screen stays blank.
    ret = esp_lcd_dpi_panel_enable_dma2d(lcd_handles_.panel);
    if (ret != ESP_OK) {
      logger_.error("Failed to enable DMA2D on DPI panel: {}", esp_err_to_name(ret));
      return false;
    }
#endif
  }

  // Send the panel controller's vendor init sequence over DBI (command mode),
  // before starting the DPI video stream.
  espp::display_drivers::Config display_config{
      .panel_io = nullptr,
      .write_command = std::bind_front(&Esp32P4FunctionEvBoard::dsi_write_command, this),
      .read_command = std::bind_front(&Esp32P4FunctionEvBoard::dsi_read_command, this),
      .lcd_send_lines = nullptr,
      .reset_pin = GPIO_NUM_NC,
      .data_command_pin = GPIO_NUM_NC,
      .reset_value = false,
      .invert_colors = invert_colors,
      .swap_color_order = swap_color_order,
      .offset_x = 0,
      .offset_y = 0,
      .swap_xy = swap_xy,
      .mirror_x = mirror_x,
      .mirror_y = mirror_y,
      .mirror_portrait = false,
  };

  display_driver_.reset();
  if (display_controller_ == DisplayController::ILI9881C) {
    auto driver = std::make_shared<espp::Ili9881>(display_config);
    if (driver->initialize()) {
      display_driver_ = std::move(driver);
    }
  } else {
    auto driver = std::make_shared<espp::Ek79007>(display_config);
    if (driver->initialize()) {
      display_driver_ = std::move(driver);
    }
  }
  if (!display_driver_) {
    logger_.error("Failed to initialize {} display controller", get_display_controller_name());
    return false;
  }

  // Low-level panel init (starts the DPI video stream)
  ret = lcd_handles_.panel->init(lcd_handles_.panel);
  if (ret != ESP_OK) {
    logger_.error("Low-level panel init failed: {}", esp_err_to_name(ret));
    return false;
  }

  // Note: the raw MIPI-DSI DPI panel does not implement disp_on_off (the panel
  // is driven on by its vendor init + the DPI video stream), so we don't call
  // esp_lcd_panel_disp_on_off() here — it would just log an unsupported error.

  // Register the DPI "color transfer done" callback so LVGL flush completes
  esp_lcd_dpi_panel_event_callbacks_t cbs = {
      .on_color_trans_done = &Esp32P4FunctionEvBoard::notify_lvgl_flush_ready,
      .on_refresh_done = nullptr,
  };
  ret = esp_lcd_dpi_panel_register_event_callbacks(lcd_handles_.panel, &cbs, this);
  if (ret != ESP_OK) {
    logger_.error("Failed to register panel event callback: {}", esp_err_to_name(ret));
    return false;
  }

  logger_.info("LCD initialization completed ({})", get_display_controller_name());
  return true;
}

void Esp32P4FunctionEvBoard::apply_panel_params(DisplayController controller) {
  display_controller_ =
      (controller == DisplayController::UNKNOWN) ? default_controller_ : controller;
  panel_params_ =
      (display_controller_ == DisplayController::ILI9881C) ? ILI9881C_PARAMS : EK79007_PARAMS;
  display_width_ = panel_params_.width;
  display_height_ = panel_params_.height;
}

static uint16_t *third_buffer = nullptr;

bool Esp32P4FunctionEvBoard::initialize_display(size_t pixel_buffer_size) {
  if (pixel_buffer_size == 0) {
    pixel_buffer_size = display_width_ * 50;
  }
  logger_.info("Initializing LVGL display with pixel buffer size: {} pixels", pixel_buffer_size);
  if (!display_) {
    display_ = std::make_shared<Display<Pixel>>(
        Display<Pixel>::LvglConfig{.width = display_width_,
                                   .height = display_height_,
                                   .flush_callback =
                                       std::bind_front(&Esp32P4FunctionEvBoard::flush, this),
                                   .rotation_callback = nullptr,
                                   .rotation = rotation},
        Display<Pixel>::OledConfig{
            .set_brightness_callback =
                [this](float brightness) { this->brightness(brightness * 100.0f); },
            .get_brightness_callback = [this]() { return this->brightness() / 100.0f; }},
        Display<Pixel>::DynamicMemoryConfig{
            .pixel_buffer_size = pixel_buffer_size,
            .double_buffered = true,
            // Allocate the LVGL draw buffers in PSRAM, not internal RAM. The
            // MIPI-DSI DMA2D path can read PSRAM, and keeping these large buffers
            // out of internal SRAM leaves room for FreeRTOS/pthread task stacks
            // (which must be internal).
            .allocation_flags = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
        },
        Logger::Verbosity::WARN);
  }

  third_buffer = (uint16_t *)heap_caps_malloc(pixel_buffer_size * sizeof(uint16_t),
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  logger_.info("LVGL display initialized");
  return true;
}

size_t Esp32P4FunctionEvBoard::rotated_display_width() const {
  auto rot = lv_display_get_rotation(lv_display_get_default());
  switch (rot) {
  case LV_DISPLAY_ROTATION_90:
  case LV_DISPLAY_ROTATION_270:
    return display_height_;
  default:
    return display_width_;
  }
}

size_t Esp32P4FunctionEvBoard::rotated_display_height() const {
  auto rot = lv_display_get_rotation(lv_display_get_default());
  switch (rot) {
  case LV_DISPLAY_ROTATION_90:
  case LV_DISPLAY_ROTATION_270:
    return display_width_;
  default:
    return display_height_;
  }
}

void Esp32P4FunctionEvBoard::write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data,
                                             uint32_t user_data) {
  (void)user_data;
  if (lcd_handles_.panel == nullptr || data == nullptr) {
    return;
  }
  if (xs < 0 || ys < 0 || xe < xs || ye < ys) {
    logger_.error("write_lcd_lines: Bad region: ({},{}) to ({},{})", xs, ys, xe, ye);
    return;
  }
  esp_lcd_panel_draw_bitmap(lcd_handles_.panel, xs, ys, xe + 1, ye + 1, data);
}

void Esp32P4FunctionEvBoard::brightness(float brightness) {
  brightness = std::clamp(brightness, 0.0f, 100.0f);
  if (backlight_) {
    backlight_->set_duty(LEDC_CHANNEL_0, brightness);
  } else {
    gpio_set_level(panel_params_.backlight_io, brightness > 0.0f ? 1 : 0);
  }
}

float Esp32P4FunctionEvBoard::brightness() const {
  if (backlight_) {
    auto maybe_duty = backlight_->get_duty(LEDC_CHANNEL_0);
    if (maybe_duty.has_value())
      return maybe_duty.value();
  }
  return gpio_get_level(panel_params_.backlight_io) ? 100.0f : 0.0f;
}

void IRAM_ATTR Esp32P4FunctionEvBoard::flush(lv_display_t *disp, const lv_area_t *area,
                                             uint8_t *px_map) {
  if (lcd_handles_.panel == nullptr) {
    lv_display_flush_ready(disp);
    return;
  }

  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;

  auto rot = lv_display_get_rotation(lv_display_get_default());
  if (rot > LV_DISPLAY_ROTATION_0 && third_buffer != nullptr) {
    int32_t ww = lv_area_get_width(area);
    int32_t hh = lv_area_get_height(area);
    lv_color_format_t cf = lv_display_get_color_format(disp);
    uint32_t w_stride = lv_draw_buf_width_to_stride(ww, cf);
    uint32_t h_stride = lv_draw_buf_width_to_stride(hh, cf);
    if (rot == LV_DISPLAY_ROTATION_180) {
      lv_draw_sw_rotate(px_map, third_buffer, hh, ww, h_stride, h_stride, LV_DISPLAY_ROTATION_180,
                        cf);
    } else if (rot == LV_DISPLAY_ROTATION_90) {
      lv_draw_sw_rotate(px_map, third_buffer, ww, hh, w_stride, h_stride, LV_DISPLAY_ROTATION_90,
                        cf);
    } else if (rot == LV_DISPLAY_ROTATION_270) {
      lv_draw_sw_rotate(px_map, third_buffer, ww, hh, w_stride, h_stride, LV_DISPLAY_ROTATION_270,
                        cf);
    }
    px_map = reinterpret_cast<uint8_t *>(third_buffer);
    lv_display_rotate_area(disp, const_cast<lv_area_t *>(area));
    offsetx1 = area->x1;
    offsetx2 = area->x2;
    offsety1 = area->y1;
    offsety2 = area->y2;
  }

  esp_lcd_panel_draw_bitmap(lcd_handles_.panel, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1,
                            px_map);
}

bool IRAM_ATTR Esp32P4FunctionEvBoard::notify_lvgl_flush_ready(
    esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx) {
  (void)panel;
  (void)edata;
  auto *board = static_cast<espp::Esp32P4FunctionEvBoard *>(user_ctx);
  if (board && board->display_) {
    board->display_->notify_flush_ready();
  }
  return false;
}

void Esp32P4FunctionEvBoard::dsi_write_command(uint8_t cmd, std::span<const uint8_t> params,
                                               uint32_t /*flags*/) {
  if (!lcd_handles_.io) {
    logger_.error("DSI write_command does not have a valid IO handle");
    return;
  }
  esp_err_t err =
      esp_lcd_panel_io_tx_param(lcd_handles_.io, (int)cmd, params.data(), params.size());
  if (err != ESP_OK) {
    logger_.error("DSI write_command 0x{:02X} failed: {}", cmd, esp_err_to_name(err));
  }
}

void Esp32P4FunctionEvBoard::dsi_read_command(uint8_t cmd, std::span<uint8_t> data,
                                              uint32_t /*flags*/) {
  if (!lcd_handles_.io) {
    logger_.error("DSI read_command does not have a valid IO handle");
    return;
  }
  esp_err_t err = esp_lcd_panel_io_rx_param(lcd_handles_.io, (int)cmd, data.data(), data.size());
  if (err != ESP_OK) {
    logger_.error("DSI read_command 0x{:02X} failed: {}", cmd, esp_err_to_name(err));
  }
}

} // namespace espp
