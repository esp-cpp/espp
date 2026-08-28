#include "esp32-p4-module-dev-kit.hpp"

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
#include <thread>

#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_interface.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_ldo_regulator.h>

using namespace std::chrono_literals;

namespace espp {

bool Esp32P4ModuleDevKit::initialize_lcd() {
  logger_.info("Initializing LCD (MIPI-DSI)");

  // Select the panel params first: the DSI lane bit rate is per-panel.
  apply_panel_params(default_controller_);

  esp_err_t ret = ESP_OK;

  // Enable the MIPI DSI PHY power LDO (on-chip LDO_VO3 -> VDD_MIPI_DPHY,
  // channel 3 @ 2500 mV on the ESP32-P4-Module-DEV-KIT).
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

  // NOTE: The ESP32-P4-Module-DEV-KIT does not route a panel reset GPIO for either panel;
  // the panel is reset over DSI during its vendor init sequence, so there is no
  // hardware GPIO reset step here (unlike the ESP32-P4-Function-EV-Board).

  // The 10.1" JD9365 panel is powered/reset and backlit by an on-board I2C
  // controller (addr 0x45) on the BSP's internal I2C bus: register 0x95 is the
  // panel power/reset control and 0x96 the backlight level (0-255) on this
  // panel's controller. This power-on sequence (values and delays) matches
  // Waveshare's vendor panel component and must run before any DSI traffic
  // so the panel is powered and out of reset when the vendor init sequence is
  // sent.
  if (display_controller_ == DisplayController::JD9365) {
    // Lazily create the backlight/panel-power I2C device on the internal bus
    // (shared with brightness()).
    if (!backlight_i2c_device_) {
      std::error_code ec;
      backlight_i2c_device_ = internal_i2c_.add_device<uint8_t>(
          {
              .device_address = backlight_i2c_address,
              .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
              .scl_speed_hz = internal_i2c_.config().clk_speed,
              .log_level = espp::Logger::Verbosity::WARN,
          },
          ec);
      if (!backlight_i2c_device_) {
        logger_.error("Could not initialize panel power/backlight I2C device (0x{:02X}): {}",
                      backlight_i2c_address, ec.message());
        return false;
      }
    }
    auto write_panel_reg = [this](uint8_t reg, uint8_t value) {
      const uint8_t data[2] = {reg, value};
      std::error_code ec;
      if (!backlight_i2c_device_->write(data, sizeof(data), ec)) {
        logger_.error("Failed to write panel power controller reg 0x{:02X}: {}", reg, ec.message());
      }
    };
    write_panel_reg(0x95, 0x11); // panel power/reset control
    write_panel_reg(0x95, 0x17); // panel power/reset control
    write_panel_reg(0x96, 0x00); // backlight off while powering up
    std::this_thread::sleep_for(100ms);
    write_panel_reg(0x96, 0xFF); // backlight full on
    std::this_thread::sleep_for(1000ms);
  }

  // Create the MIPI DSI bus (also initializes the DSI PHY)
  if (lcd_handles_.mipi_dsi_bus == nullptr) {
    logger_.info("Creating MIPI DSI bus ({} lanes, {} Mbps/lane)", mipi_dsi_lanes,
                 panel_params_.lane_bitrate_mbps);
    esp_lcd_dsi_bus_config_t bus_config = {};
    bus_config.bus_id = 0;
    bus_config.num_data_lanes = mipi_dsi_lanes;
    bus_config.phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT;
    bus_config.lane_bit_rate_mbps = panel_params_.lane_bitrate_mbps;
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
  // timing).
  logger_.info("Using display panel: {} ({}x{})", get_display_controller_name(), display_width_,
               display_height_);

  // NOTE: The ESP32-P4-Module-DEV-KIT has no backlight GPIO. The backlight is driven by an
  // on-board I2C controller (addr 0x45). On the 10.1" JD9365 panel brightness()
  // writes that controller (reg 0x96); on the other panels the panel powers up
  // with the backlight on and brightness() only stores the value. No espp::Led /
  // PWM backlight is instantiated here.
  brightness(100.0f);

  // espp-driver path (JD9365 / ILI9881C / EK79007): send the panel
  // controller's vendor init sequence over DBI (command mode), before starting
  // the DPI video stream.
  espp::display_drivers::Config display_config{
      .panel_io = nullptr,
      .write_command = std::bind_front(&Esp32P4ModuleDevKit::dsi_write_command, this),
      // NOTE: the Waveshare ESP32-P4 panels do not reliably support MIPI-DSI DCS
      // reads (bus turn-around); the ESP-IDF HAL busy-waits on the read, which
      // hangs panel init and trips the task watchdog. Do not provide a
      // read_command so the driver skips the optional panel-ID read.
      .read_command = nullptr,
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
  if (display_controller_ == DisplayController::JD9365) {
    // The Waveshare 10.1" 800x1280 panel is a JD9365. espp::Jd9365 performs
    // the DCS software reset and sends the vendor init sequence (taken from
    // Waveshare's vendor panel component; see jd9365.hpp) over the DBI IO.
    auto driver = std::make_shared<espp::Jd9365>(display_config);
    if (driver->initialize()) {
      display_driver_ = std::move(driver);
    }
  } else if (display_controller_ == DisplayController::ILI9881C) {
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

  // Create the DPI (video) panel with the configured panel's timing. This must
  // come AFTER the vendor init sequence above: esp_lcd_new_panel_dpi() starts the
  // HS video stream, and once it is running the DSI cannot drain the low-power
  // command FIFO, so a long init sequence (e.g. ILI9881C's 202 commands) would
  // overflow it and hang.
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
    // NOTE: for the ILI9881C / EK79007 panels we deliberately do NOT enable
    // DMA2D for the DPI panel. DMA2D is a color-processing engine, not a plain
    // copy: routing the LVGL flush (esp_lcd_panel_draw_bitmap) through it
    // corrupts the RGB565 channel order on those panels, while the plain CPU
    // copy path renders correctly. The JD9365 panel renders correctly WITH
    // DMA2D (and Waveshare's vendor panel component enables it), so keep it
    // enabled on that path.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    if (display_controller_ == DisplayController::JD9365) {
      ret = esp_lcd_dpi_panel_enable_dma2d(lcd_handles_.panel);
      if (ret != ESP_OK) {
        logger_.error("Failed to enable DMA2D for the DPI panel: {}", esp_err_to_name(ret));
        return false;
      }
    }
#endif
  }

  // Low-level panel init (starts the DPI video stream)
  ret = lcd_handles_.panel->init(lcd_handles_.panel);
  if (ret != ESP_OK) {
    logger_.error("Low-level panel init failed: {}", esp_err_to_name(ret));
    return false;
  }

  // Note: the raw MIPI-DSI DPI panel does not implement disp_on_off (the panel
  // is driven on by its vendor init + the DPI video stream), so we don't call
  // esp_lcd_panel_disp_on_off() here.

  // Register the DPI "color transfer done" callback so LVGL flush completes
  esp_lcd_dpi_panel_event_callbacks_t cbs = {
      .on_color_trans_done = &Esp32P4ModuleDevKit::notify_lvgl_flush_ready,
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

void Esp32P4ModuleDevKit::apply_panel_params(DisplayController controller) {
  display_controller_ =
      (controller == DisplayController::UNKNOWN) ? default_controller_ : controller;
  switch (display_controller_) {
  case DisplayController::JD9365:
    panel_params_ = JD9365_PARAMS;
    break;
  case DisplayController::ILI9881C:
    panel_params_ = ILI9881C_PARAMS;
    break;
  default:
    panel_params_ = EK79007_PARAMS;
    break;
  }
  display_width_ = panel_params_.width;
  display_height_ = panel_params_.height;
}

static uint16_t *third_buffer = nullptr;
static size_t third_buffer_px = 0; // allocated capacity of third_buffer, in pixels

bool Esp32P4ModuleDevKit::initialize_display(size_t pixel_buffer_size) {
  if (pixel_buffer_size == 0) {
    pixel_buffer_size = display_width_ * 50;
  }
  logger_.info("Initializing LVGL display with pixel buffer size: {} pixels", pixel_buffer_size);
  if (!display_) {
    display_ = std::make_shared<Display<Pixel>>(
        Display<Pixel>::LvglConfig{.width = display_width_,
                                   .height = display_height_,
                                   .flush_callback =
                                       std::bind_front(&Esp32P4ModuleDevKit::flush, this),
                                   .rotation_callback = nullptr,
                                   .rotation = rotation},
        Display<Pixel>::OledConfig{
            .set_brightness_callback =
                [this](float brightness) { this->brightness(brightness * 100.0f); },
            .get_brightness_callback = [this]() { return this->brightness() / 100.0f; }},
        Display<Pixel>::DynamicMemoryConfig{
            .pixel_buffer_size = pixel_buffer_size,
            .double_buffered = true,
            // Allocate the LVGL draw buffers in PSRAM to keep these large buffers
            // out of internal SRAM. The CPU-copy flush reads them coherently.
            .allocation_flags = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
        },
        Logger::Verbosity::WARN);
  }

  // Rotation scratch buffer (only used when the display is rotated). Allocate it
  // once and reuse it across re-inits so re-initialization does not leak PSRAM;
  // if it fails, flush() detects the null pointer and simply skips rotation.
  if (third_buffer == nullptr || pixel_buffer_size > third_buffer_px) {
    if (third_buffer != nullptr) {
      heap_caps_free(third_buffer);
      third_buffer = nullptr;
      third_buffer_px = 0;
    }
    third_buffer = (uint16_t *)heap_caps_malloc(pixel_buffer_size * sizeof(uint16_t),
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    third_buffer_px = (third_buffer != nullptr) ? pixel_buffer_size : 0;
    if (third_buffer == nullptr) {
      logger_.warn("Could not allocate the {}-byte display rotation buffer; rotation disabled",
                   pixel_buffer_size * sizeof(uint16_t));
    }
  }

  logger_.info("LVGL display initialized");
  return true;
}

size_t Esp32P4ModuleDevKit::rotated_display_width() const {
  auto rot = lv_display_get_rotation(lv_display_get_default());
  switch (rot) {
  case LV_DISPLAY_ROTATION_90:
  case LV_DISPLAY_ROTATION_270:
    return display_height_;
  default:
    return display_width_;
  }
}

size_t Esp32P4ModuleDevKit::rotated_display_height() const {
  auto rot = lv_display_get_rotation(lv_display_get_default());
  switch (rot) {
  case LV_DISPLAY_ROTATION_90:
  case LV_DISPLAY_ROTATION_270:
    return display_width_;
  default:
    return display_height_;
  }
}

void Esp32P4ModuleDevKit::write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data,
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

void Esp32P4ModuleDevKit::brightness(float brightness) {
  // The ESP32-P4-Module-DEV-KIT has NO backlight GPIO. The backlight is driven by an
  // on-board I2C controller at address 0x45. On the 10.1" JD9365 panel the
  // brightness register is 0x96 (value 0-255); Waveshare's own BSP writes 0x86
  // instead for some panel revisions. For panels other than the JD9365 the
  // chip/protocol has not been verified, so this remains best-effort: store
  // the requested value so brightness() reads back what was set, and log it.
  brightness = std::clamp(brightness, 0.0f, 100.0f);
  brightness_ = brightness;
  if (display_controller_ != DisplayController::JD9365) {
    logger_.debug("brightness({}) requested; no backlight GPIO on this board (on-board I2C "
                  "backlight protocol not verified for this panel), value stored only",
                  brightness);
    return;
  }
  // Lazily create the backlight I2C device on the internal bus.
  if (!backlight_i2c_device_) {
    std::error_code ec;
    backlight_i2c_device_ = internal_i2c_.add_device<uint8_t>(
        {
            .device_address = backlight_i2c_address,
            .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
            .scl_speed_hz = internal_i2c_.config().clk_speed,
            .log_level = espp::Logger::Verbosity::WARN,
        },
        ec);
    if (!backlight_i2c_device_) {
      logger_.error("Could not initialize backlight I2C device (0x{:02X}): {}",
                    backlight_i2c_address, ec.message());
      return;
    }
  }
  // Register 0x96 is the brightness register (0-255) on the 10.1" JD9365
  // panel's power/backlight controller (Waveshare's BSP uses 0x86 for some
  // panel revisions).
  const uint8_t data[2] = {0x96, static_cast<uint8_t>(255.0f * brightness / 100.0f)};
  std::error_code ec;
  if (!backlight_i2c_device_->write(data, sizeof(data), ec)) {
    logger_.error("Failed to write backlight brightness: {}", ec.message());
  }
}

float Esp32P4ModuleDevKit::brightness() const { return brightness_.load(); }

void IRAM_ATTR Esp32P4ModuleDevKit::flush(lv_display_t *disp, const lv_area_t *area,
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
  int32_t ww = lv_area_get_width(area);
  int32_t hh = lv_area_get_height(area);
  // Only rotate when the rotated area fits the scratch buffer; otherwise skip
  // rotation for this flush (fall back to unrotated) to avoid overflowing it.
  if (rot > LV_DISPLAY_ROTATION_0 && third_buffer != nullptr &&
      static_cast<size_t>(ww) * static_cast<size_t>(hh) <= third_buffer_px) {
    lv_color_format_t cf = lv_display_get_color_format(disp);
    uint32_t w_stride = lv_draw_buf_width_to_stride(ww, cf);
    uint32_t h_stride = lv_draw_buf_width_to_stride(hh, cf);
    if (rot == LV_DISPLAY_ROTATION_180) {
      // 180° keeps the source dimensions (unlike 90/270), so pass ww/hh and the
      // width-based stride for both source and destination.
      lv_draw_sw_rotate(px_map, third_buffer, ww, hh, w_stride, w_stride, LV_DISPLAY_ROTATION_180,
                        cf);
    } else if (rot == LV_DISPLAY_ROTATION_90) {
      lv_draw_sw_rotate(px_map, third_buffer, ww, hh, w_stride, h_stride, LV_DISPLAY_ROTATION_90,
                        cf);
    } else if (rot == LV_DISPLAY_ROTATION_270) {
      lv_draw_sw_rotate(px_map, third_buffer, ww, hh, w_stride, h_stride, LV_DISPLAY_ROTATION_270,
                        cf);
    }
    px_map = reinterpret_cast<uint8_t *>(third_buffer);
    // Rotate a local copy of the area; LVGL provides a const area* and mutating
    // it via const_cast would be undefined behavior.
    lv_area_t rotated = *area;
    lv_display_rotate_area(disp, &rotated);
    offsetx1 = rotated.x1;
    offsetx2 = rotated.x2;
    offsety1 = rotated.y1;
    offsety2 = rotated.y2;
  }

  esp_lcd_panel_draw_bitmap(lcd_handles_.panel, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1,
                            px_map);
}

bool IRAM_ATTR Esp32P4ModuleDevKit::notify_lvgl_flush_ready(esp_lcd_panel_handle_t panel,
                                                            esp_lcd_dpi_panel_event_data_t *edata,
                                                            void *user_ctx) {
  (void)panel;
  (void)edata;
  auto *board = static_cast<espp::Esp32P4ModuleDevKit *>(user_ctx);
  if (board && board->display_) {
    board->display_->notify_flush_ready();
  }
  return false;
}

void Esp32P4ModuleDevKit::dsi_write_command(uint8_t cmd, std::span<const uint8_t> params,
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

void Esp32P4ModuleDevKit::dsi_read_command(uint8_t cmd, std::span<uint8_t> data,
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
