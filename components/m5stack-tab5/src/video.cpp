#include "m5stack-tab5.hpp"

#include "esp_idf_version.h"
#ifndef ESP_IDF_VERSION_VAL
#define ESP_IDF_VERSION_VAL(major, minor, patch) (((major) << 16) | ((minor) << 8) | (patch))
#endif
#ifndef ESP_IDF_VERSION
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(0, 0, 0)
#endif

#include <algorithm>
#include <cstdlib>

#include <driver/ppa.h>
#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_ldo_regulator.h>

using namespace std::chrono_literals;

namespace espp {

M5StackTab5::DisplayController M5StackTab5::detect_display_controller() {
  auto &i2c = internal_i2c();

  // The Tab5 has shipped with three display revisions:
  //   - ILI9881C panel + separate GT911 touch controller (original),
  //   - ST7123 TDDI (integrated display+touch, post Oct-2025),
  //   - ST7121 TDDI (newest).
  // The two Sitronix TDDI parts answer on the same touch I2C address (0x55)
  // and cannot be told apart by their DSI ID, but they need DIFFERENT init
  // sequences and DSI lane rates — mis-identifying one as the other yields a
  // black screen. Distinguish them by the touch firmware-version register
  // (2-byte register address 0x0000): 1 = ST7121, 3 = ST7123 (same method
  // M5GFX uses). The ST touch engine can take ~50 ms after reset before it
  // responds, so poll rather than probing once; a GT911 ACK at any point
  // identifies the ILI9881 variant immediately.
  static constexpr uint8_t kStTouchAddress = 0x55;
  static constexpr uint8_t kGt911Address = 0x14; // TouchDriver::DEFAULT_ADDRESS_2
  bool st_acked = false;
  int last_logged_fw = -1;
  for (int attempt = 0; attempt < 60; ++attempt) {
    const uint8_t fw_reg[2] = {0x00, 0x00};
    uint8_t fw_version = 0;
    if (i2c.write_read(kStTouchAddress, fw_reg, sizeof(fw_reg), &fw_version, 1)) {
      st_acked = true;
      if (fw_version != last_logged_fw) {
        last_logged_fw = fw_version;
        logger_.info("ST touch firmware version: {:#04x}", fw_version);
      }
      if (fw_version == 1) {
        return M5StackTab5::DisplayController::ST7121;
      }
      if (fw_version == 3) {
        return M5StackTab5::DisplayController::ST7123;
      }
      // Unknown FW value — the touch engine may still be booting; keep polling.
    } else if (i2c.probe_device(kGt911Address)) {
      // GT911 present -> original ILI9881 revision.
      return M5StackTab5::DisplayController::ILI9881;
    }
    std::this_thread::sleep_for(10ms);
  }

  if (st_acked) {
    // An ST TDDI is present but reported an unrecognized firmware version.
    // Fall back to the ST7123 (the more common part) rather than failing.
    logger_.warn("ST touch controller present but firmware version {:#04x} is unknown; "
                 "assuming ST7123",
                 last_logged_fw);
    return M5StackTab5::DisplayController::ST7123;
  }

  // Unknown display controller
  return M5StackTab5::DisplayController::UNKNOWN;
}
bool M5StackTab5::initialize_lcd() {
  logger_.info("Initializing M5Stack Tab5 LCD (MIPI-DSI, {}x{})", display_width_, display_height_);

  if (!ioexp_0x43_) {
    if (!initialize_io_expanders()) {
      logger_.error("Failed to init IO expanders for LCD reset");
      return false;
    }
  }

  esp_err_t ret = ESP_OK;

  // enable DSI PHY power
  static esp_ldo_channel_handle_t phy_pwr_chan = nullptr;
  {
    logger_.info("Acquiring MIPI DSI PHY power LDO channel");
    esp_ldo_channel_config_t phy_pwr_cfg{};
    memset(&phy_pwr_cfg, 0, sizeof(phy_pwr_cfg));
    static constexpr int MIPI_DSI_PHY_PWR_LDO_CHANNEL = 3;
    static constexpr int MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV = 2500;
    phy_pwr_cfg.chan_id = MIPI_DSI_PHY_PWR_LDO_CHANNEL;
    phy_pwr_cfg.voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV;
    ret = esp_ldo_acquire_channel(&phy_pwr_cfg, &phy_pwr_chan);
    if (ret != ESP_OK) {
      logger_.error("Failed to acquire MIPI DSI PHY power LDO channel: {}", esp_err_to_name(ret));
      return false;
    }
  }

  // Configure backlight PWM
  if (!backlight_) {
    backlight_channel_configs_.push_back({.gpio = static_cast<size_t>(lcd_backlight_io),
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

  // default to 100% brightness to ensure users can see screen
  brightness(100.0f);

  // Perform hardware reset sequence via IO expander
  logger_.info("Performing LCD hardware reset sequence");
  lcd_reset(true); // Assert reset
  std::this_thread::sleep_for(10ms);
  lcd_reset(false); // Release reset
  std::this_thread::sleep_for(120ms);

  // Detect and initialize the appropriate display controller
  logger_.info("Detecting display controller type");
  auto detected_controller = detect_display_controller();

  // create MIPI DSI bus first, it will initialize the DSI PHY as well
  if (lcd_handles_.mipi_dsi_bus == nullptr) {
    logger_.info("Creating MIPI DSI bus");
    esp_lcd_dsi_bus_config_t bus_config = {};
    bus_config.bus_id = 0;
    bus_config.num_data_lanes = 2;
    bus_config.phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT;
    // Per-controller DSI lane rate. The ST7121 runs a slower lane clock than
    // the ST7123 (M5GFX reference: 900 vs 1040 Mbps); driving it at the
    // ST7123 rate is one of the reasons a mis-detected ST7121 shows nothing.
    switch (detected_controller) {
    case DisplayController::ILI9881:
      bus_config.lane_bit_rate_mbps = 730;
      break;
    case DisplayController::ST7121:
      bus_config.lane_bit_rate_mbps = 900;
      break;
    default: // ST7123 (and the ST fallback)
      bus_config.lane_bit_rate_mbps = 965;
      break;
    }
    ret = esp_lcd_new_dsi_bus(&bus_config, &lcd_handles_.mipi_dsi_bus);
    if (ret != ESP_OK) {
      logger_.error("New DSI bus init failed: {}", esp_err_to_name(ret));
      return false;
    }
  }

  if (lcd_handles_.io == nullptr) {
    logger_.info("Install MIPI DSI LCD panel I/O");
    // we use DBI interface to send LCD commands and parameters
    esp_lcd_dbi_io_config_t dbi_config = {};
    dbi_config.virtual_channel = 0;
    dbi_config.lcd_cmd_bits = 8;
    dbi_config.lcd_param_bits = 8;
    ret = esp_lcd_new_panel_io_dbi(lcd_handles_.mipi_dsi_bus, &dbi_config, &lcd_handles_.io);
    if (ret != ESP_OK) {
      logger_.error("New panel IO failed: {}", esp_err_to_name(ret));
      // TODO: free previously allocated resources
      return false;
    }
  }

  if (detected_controller == DisplayController::UNKNOWN) {
    logger_.error("Unable to detect display controller");
    return false;
  }
  logger_.info("Detected display controller: {}", get_display_controller_name(detected_controller));

  esp_lcd_dpi_panel_config_t dpi_cfg{};
  memset(&dpi_cfg, 0, sizeof(dpi_cfg));

  if (detected_controller == DisplayController::ILI9881 && lcd_handles_.panel == nullptr) {
    // Create DPI panel with M5Stack Tab5 official ILI9881 timing parameters
    logger_.info("Creating MIPI DSI DPI panel with M5Stack Tab5 ILI9881 configuration");
    dpi_cfg.virtual_channel = 0;
    dpi_cfg.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    dpi_cfg.dpi_clock_freq_mhz = 60;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    dpi_cfg.in_color_format = LCD_COLOR_FMT_RGB565;
    dpi_cfg.out_color_format = LCD_COLOR_FMT_RGB565;
#else
    dpi_cfg.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
#endif
    dpi_cfg.num_fbs = 1;
    dpi_cfg.video_timing.h_size = display_width_;
    dpi_cfg.video_timing.v_size = display_height_;
    dpi_cfg.video_timing.hsync_back_porch = 140;
    dpi_cfg.video_timing.hsync_pulse_width = 40;
    dpi_cfg.video_timing.hsync_front_porch = 40;
    dpi_cfg.video_timing.vsync_back_porch = 20;
    dpi_cfg.video_timing.vsync_pulse_width = 4;
    dpi_cfg.video_timing.vsync_front_porch = 20;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
    dpi_cfg.flags.use_dma2d = true;
#endif

  } else if (detected_controller == DisplayController::ST7123 && lcd_handles_.panel == nullptr) {
    dpi_cfg.virtual_channel = 0;
    dpi_cfg.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    // The ST7123 is a TDDI part: its touch engine scans during the display
    // blanking interval and is timed against the pixel clock the vendor init
    // table was tuned for. Running the panel faster than the reference 70 MHz
    // (M5Stack/esp-bsp value) shrinks the blanking window and desyncs the touch
    // scan, so the panel shows but touch never reports. Keep this at 70 MHz.
    dpi_cfg.dpi_clock_freq_mhz = 70;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    dpi_cfg.in_color_format = LCD_COLOR_FMT_RGB565;
    dpi_cfg.out_color_format = LCD_COLOR_FMT_RGB565;
#else
    dpi_cfg.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
#endif
    dpi_cfg.num_fbs = 1;
    dpi_cfg.video_timing.h_size = display_width_;
    dpi_cfg.video_timing.v_size = display_height_;
    dpi_cfg.video_timing.hsync_back_porch = 40;
    dpi_cfg.video_timing.hsync_pulse_width = 2;
    dpi_cfg.video_timing.hsync_front_porch = 40;
    dpi_cfg.video_timing.vsync_back_porch = 8;
    dpi_cfg.video_timing.vsync_pulse_width = 2;
    dpi_cfg.video_timing.vsync_front_porch = 220;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
    dpi_cfg.flags.use_dma2d = true;
#endif
  } else if (detected_controller == DisplayController::ST7121 && lcd_handles_.panel == nullptr) {
    dpi_cfg.virtual_channel = 0;
    dpi_cfg.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    // Like the ST7123, the ST7121 is a TDDI part whose touch engine is timed
    // against the pixel clock; use the M5GFX reference 70 MHz and porch set
    // (they differ from the ST7123's: VBP 24 / VPW 20 / VFP 200).
    dpi_cfg.dpi_clock_freq_mhz = 70;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    dpi_cfg.in_color_format = LCD_COLOR_FMT_RGB565;
    dpi_cfg.out_color_format = LCD_COLOR_FMT_RGB565;
#else
    dpi_cfg.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
#endif
    dpi_cfg.num_fbs = 1;
    dpi_cfg.video_timing.h_size = display_width_;
    dpi_cfg.video_timing.v_size = display_height_;
    dpi_cfg.video_timing.hsync_back_porch = 40;
    dpi_cfg.video_timing.hsync_pulse_width = 2;
    dpi_cfg.video_timing.hsync_front_porch = 40;
    dpi_cfg.video_timing.vsync_back_porch = 24;
    dpi_cfg.video_timing.vsync_pulse_width = 20;
    dpi_cfg.video_timing.vsync_front_porch = 200;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
    dpi_cfg.flags.use_dma2d = true;
#endif
  }

  if (lcd_handles_.panel == nullptr) {
    logger_.info("Creating DPI panel with resolution {}x{}", dpi_cfg.video_timing.h_size,
                 dpi_cfg.video_timing.v_size);
    ret = esp_lcd_new_panel_dpi(lcd_handles_.mipi_dsi_bus, &dpi_cfg, &lcd_handles_.panel);
    if (ret != ESP_OK) {
      logger_.error("Failed to create MIPI DSI DPI panel: {}", esp_err_to_name(ret));
      return false;
    }
  }

  espp::display_drivers::Config display_config{
      .panel_io = nullptr,
      .write_command = std::bind_front(&M5StackTab5::dsi_write_command, this),
      .read_command = std::bind_front(&M5StackTab5::dsi_read_command, this),
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
  if (detected_controller == DisplayController::ILI9881) {
    logger_.info("Initializing as ILI9881");
    auto display_driver = std::make_shared<espp::Ili9881>(display_config);
    if (display_driver->initialize()) {
      logger_.info("Successfully initialized ILI9881 display controller");
      display_driver_ = std::move(display_driver);
      display_controller_ = DisplayController::ILI9881;
    }
  } else if (detected_controller == DisplayController::ST7123) {
    logger_.info("Initializing as ST7123");
    auto display_driver = std::make_shared<espp::St7123>(display_config);
    if (display_driver->initialize()) {
      logger_.info("Successfully initialized ST7123 display controller");
      display_driver_ = std::move(display_driver);
      display_controller_ = DisplayController::ST7123;
    }
  } else if (detected_controller == DisplayController::ST7121) {
    logger_.info("Initializing as ST7121");
    auto display_driver = std::make_shared<espp::St7121>(display_config);
    if (display_driver->initialize()) {
      logger_.info("Successfully initialized ST7121 display controller");
      display_driver_ = std::move(display_driver);
      display_controller_ = DisplayController::ST7121;
    }
  } else {
    logger_.error("Failed to detect display controller");
    return false;
  }

  if (!display_driver_) {
    logger_.error("Failed to initialize {} display controller",
                  get_display_controller_name(detected_controller));
    return false;
  }

  logger_.info("Display controller: {}", get_display_controller_name());

  // call init on the panel
  logger_.info("Calling low-level panel init");
  ret = lcd_handles_.panel->init(lcd_handles_.panel);
  if (ret != ESP_OK) {
    logger_.error("Low-level panel init failed: {}", esp_err_to_name(ret));
    return false;
  }

  logger_.info("Display initialized with resolution {}x{}", display_width_, display_height_);

  logger_.info("Register DPI panel event callback for LVGL flush ready notification");
  esp_lcd_dpi_panel_event_callbacks_t cbs = {
      .on_color_trans_done = &M5StackTab5::notify_lvgl_flush_ready,
      .on_refresh_done = nullptr,
  };
  ret = esp_lcd_dpi_panel_register_event_callbacks(lcd_handles_.panel, &cbs, this);
  if (ret != ESP_OK) {
    logger_.error("Failed to register panel event callback: {}", esp_err_to_name(ret));
    return false;
  }

  logger_.info("M5Stack Tab5 LCD initialization completed successfully");
  return true;
}

// Scratch buffer that holds the hardware-rotated frame produced by the PPA
// before it is handed to the panel (see flush()). Kept in PSRAM and aligned to
// the data-cache line size, which the PPA requires for its output buffer.
static uint16_t *third_buffer = nullptr;
static size_t third_buffer_bytes = 0;
// PPA (Pixel Processing Accelerator) client used to rotate the frame in
// hardware instead of on the CPU (lv_draw_sw_rotate). CPU rotation of a
// full-frame PSRAM buffer is slow and, because the transpose strides PSRAM,
// asymmetric between 90 and 270 degrees; the PPA is fast and symmetric.
static ppa_client_handle_t g_ppa_client = nullptr;

bool M5StackTab5::initialize_display(size_t pixel_buffer_size) {
  logger_.info("Initializing LVGL display with pixel buffer size: {} pixels", pixel_buffer_size);
  if (!display_) {
    display_ = std::make_shared<Display<Pixel>>(
        Display<Pixel>::LvglConfig{.width = display_width_,
                                   .height = display_height_,
                                   .flush_callback = std::bind_front(&M5StackTab5::flush, this),
                                   .rotation_callback = nullptr, // DisplayDriver::rotate,
                                   .rotation = rotation},
        Display<Pixel>::OledConfig{
            .set_brightness_callback =
                [this](float brightness) { this->brightness(brightness * 100.0f); },
            .get_brightness_callback = [this]() { return this->brightness() / 100.0f; }},
        Display<Pixel>::DynamicMemoryConfig{
            .pixel_buffer_size = pixel_buffer_size,
            .double_buffered = true,
            // Allocate the LVGL draw buffers in PSRAM: a full-frame double
            // buffer is ~3.7 MB, far larger than internal RAM, and the DPI
            // panel's draw_bitmap reads the source straight from PSRAM (the
            // esp32-p4-function-ev-board uses the same PSRAM buffers).
            .allocation_flags = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
        },
        Logger::Verbosity::WARN);
  }

  // Register a PPA client for hardware rotation, and allocate the rotation
  // scratch buffer that the PPA writes into. The PPA output buffer (external /
  // PSRAM memory) must be aligned to the data cache line size, and both the
  // pointer and the size must be a multiple of it.
  if (g_ppa_client == nullptr) {
    ppa_client_config_t ppa_cfg = {};
    ppa_cfg.oper_type = PPA_OPERATION_SRM;
    esp_err_t perr = ppa_register_client(&ppa_cfg, &g_ppa_client);
    if (perr != ESP_OK) {
      logger_.warn("Could not register PPA client ({}); rotation will fall back to software",
                   esp_err_to_name(perr));
      g_ppa_client = nullptr;
    }
  }
  // Align to 128 bytes: the PPA output buffer in external (PSRAM) memory must
  // be aligned to the L1 and L2 cache line size, and the ESP32-P4's L2 line is
  // 128 bytes. Both the pointer and the size must be a multiple of it.
  static constexpr size_t kCacheAlign = 128;
  size_t required_bytes = pixel_buffer_size * sizeof(uint16_t);
  required_bytes = (required_bytes + kCacheAlign - 1) / kCacheAlign * kCacheAlign;

  // Reuse the existing scratch buffer if it is already the right size; otherwise
  // free it first so a repeated initialize_display() call (e.g. re-init with a
  // different pixel buffer size) cannot leak the previous allocation.
  if (third_buffer == nullptr || third_buffer_bytes != required_bytes) {
    if (third_buffer != nullptr) {
      heap_caps_free(third_buffer);
      third_buffer = nullptr;
    }
    third_buffer_bytes = required_bytes;
    third_buffer = (uint16_t *)heap_caps_aligned_alloc(kCacheAlign, third_buffer_bytes,
                                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (third_buffer == nullptr) {
      // The scratch buffer is required for display rotation - both the PPA path
      // and the software fallback rotate into it - so without it a non-zero
      // rotation (settable at runtime) would silently flush incorrect output.
      // Fail initialization rather than come up with rotation quietly broken.
      logger_.error("Could not allocate the rotation scratch buffer ({} bytes)", required_bytes);
      third_buffer_bytes = 0;
      return false;
    }
  }

  logger_.info("LVGL display initialized");
  return true;
}

size_t M5StackTab5::rotated_display_width() const {
  auto rotation = lv_display_get_rotation(lv_display_get_default());
  switch (rotation) {
  // swap
  case LV_DISPLAY_ROTATION_90:
  case LV_DISPLAY_ROTATION_270:
    return display_height_;
  // as configured
  case LV_DISPLAY_ROTATION_0:
  case LV_DISPLAY_ROTATION_180:
  default:
    return display_width_;
  }
}

size_t M5StackTab5::rotated_display_height() const {
  auto rotation = lv_display_get_rotation(lv_display_get_default());
  switch (rotation) {
  // swap
  case LV_DISPLAY_ROTATION_90:
  case LV_DISPLAY_ROTATION_270:
    return display_width_;
  // as configured
  case LV_DISPLAY_ROTATION_0:
  case LV_DISPLAY_ROTATION_180:
  default:
    return display_height_;
  }
}

void M5StackTab5::write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data,
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

void M5StackTab5::brightness(float brightness) {
  brightness = std::clamp(brightness, 0.0f, 100.0f);
  if (backlight_) {
    backlight_->set_duty(LEDC_CHANNEL_0, brightness);
  } else {
    gpio_set_level(lcd_backlight_io, brightness > 0.0f ? 1 : 0);
  }
}

float M5StackTab5::brightness() const {
  if (backlight_) {
    auto maybe_duty = backlight_->get_duty(LEDC_CHANNEL_0);
    if (maybe_duty.has_value())
      return maybe_duty.value();
  }
  return gpio_get_level(lcd_backlight_io) ? 100.0f : 0.0f;
}

// -----------------
// DSI write helpers
// -----------------

void M5StackTab5::flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  // This is LVGL's flush callback; it runs in the LVGL task context (from
  // lv_display_flush / the LVGL timer), not from an ISR - the DPI transfer-done
  // interrupt is handled separately in notify_lvgl_flush_ready(). Blocking work
  // (the PPA rotation and esp_lcd_panel_draw_bitmap) is therefore safe here.

  if (lcd_handles_.panel == nullptr) {
    lv_display_flush_ready(disp);
    return;
  }

  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;

  auto rotation = lv_display_get_rotation(lv_display_get_default());
  // Cache the rotation for the camera task, which must not call LVGL from its
  // own thread. flush() runs on the LVGL (GUI) thread, so reading it here is
  // safe; the camera task reads the cached atomic instead.
  camera_display_rotation_.store(static_cast<uint8_t>(rotation), std::memory_order_relaxed);
  if (rotation > LV_DISPLAY_ROTATION_0 && third_buffer != nullptr) {
    int32_t ww = lv_area_get_width(area);
    int32_t hh = lv_area_get_height(area);
    lv_color_format_t cf = lv_display_get_color_format(disp);
    bool rotated = false;
    if (g_ppa_client != nullptr) {
      // Hardware rotation via the PPA. The LVGL rotation maps directly onto the
      // PPA rotation angle (LVGL 90 -> PPA 90, 180 -> 180, 270 -> 270); this is
      // the mapping verified on hardware. For 90/270 the output picture
      // width/height are swapped. If a different panel comes out turned the
      // wrong way, swap the 90 and 270 cases here.
      ppa_srm_rotation_angle_t angle = PPA_SRM_ROTATION_ANGLE_0;
      uint32_t out_w = ww, out_h = hh;
      if (rotation == LV_DISPLAY_ROTATION_90) {
        angle = PPA_SRM_ROTATION_ANGLE_90;
        out_w = hh;
        out_h = ww;
      } else if (rotation == LV_DISPLAY_ROTATION_180) {
        angle = PPA_SRM_ROTATION_ANGLE_180;
      } else if (rotation == LV_DISPLAY_ROTATION_270) {
        angle = PPA_SRM_ROTATION_ANGLE_270;
        out_w = hh;
        out_h = ww;
      }
      ppa_srm_oper_config_t srm = {};
      srm.in.buffer = px_map;
      srm.in.pic_w = ww;
      srm.in.pic_h = hh;
      srm.in.block_w = ww;
      srm.in.block_h = hh;
      srm.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
      srm.out.buffer = third_buffer;
      srm.out.buffer_size = third_buffer_bytes;
      srm.out.pic_w = out_w;
      srm.out.pic_h = out_h;
      srm.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
      srm.rotation_angle = angle;
      srm.scale_x = 1.0f;
      srm.scale_y = 1.0f;
      srm.mode = PPA_TRANS_MODE_BLOCKING;
      // On failure third_buffer holds stale/partial data; leave rotated=false so
      // we fall through to the software rotation below rather than flushing it.
      rotated = (ppa_do_scale_rotate_mirror(g_ppa_client, &srm) == ESP_OK);
    }
    if (!rotated) {
      // Software fallback: the PPA client failed to register or the PPA
      // operation failed. Rotates into third_buffer, fully overwriting it.
      uint32_t w_stride = lv_draw_buf_width_to_stride(ww, cf);
      uint32_t h_stride = lv_draw_buf_width_to_stride(hh, cf);
      if (rotation == LV_DISPLAY_ROTATION_180) {
        lv_draw_sw_rotate(px_map, third_buffer, hh, ww, h_stride, h_stride, LV_DISPLAY_ROTATION_180,
                          cf);
      } else if (rotation == LV_DISPLAY_ROTATION_90) {
        lv_draw_sw_rotate(px_map, third_buffer, ww, hh, w_stride, h_stride, LV_DISPLAY_ROTATION_90,
                          cf);
      } else if (rotation == LV_DISPLAY_ROTATION_270) {
        lv_draw_sw_rotate(px_map, third_buffer, ww, hh, w_stride, h_stride, LV_DISPLAY_ROTATION_270,
                          cf);
      }
    }
    px_map = reinterpret_cast<uint8_t *>(third_buffer);
    lv_display_rotate_area(disp, const_cast<lv_area_t *>(area));
    offsetx1 = area->x1;
    offsetx2 = area->x2;
    offsety1 = area->y1;
    offsety2 = area->y2;
  }

  // pass the draw buffer to the DPI panel driver
  esp_lcd_panel_draw_bitmap(lcd_handles_.panel, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1,
                            px_map);
  // For DPI panels, the notification will come through the callback
}

bool IRAM_ATTR M5StackTab5::notify_lvgl_flush_ready(esp_lcd_panel_handle_t panel,
                                                    esp_lcd_dpi_panel_event_data_t *edata,
                                                    void *user_ctx) {
  espp::M5StackTab5 *tab5 = static_cast<espp::M5StackTab5 *>(user_ctx);
  if (tab5 == nullptr) {
    return false;
  }

  // This is called from ISR context, so we need to be careful about what we do
  // Just notify LVGL that the flush is ready - avoid logging or other complex operations
  if (tab5->display_) {
    tab5->display_->notify_flush_ready();
  }
  return false;
}

void M5StackTab5::dsi_write_command(uint8_t cmd, std::span<const uint8_t> params,
                                    uint32_t /*flags*/) {
  if (!lcd_handles_.io) {
    logger_.error("DSI write_command does not have a valid IO handle");
    return;
  }

  // logger_.debug("DSI write_command 0x{:02X} with {} bytes", cmd, params.size());

  esp_lcd_panel_io_handle_t io = lcd_handles_.io;
  const void *data_ptr = params.data();
  size_t data_size = params.size();
  esp_err_t err = esp_lcd_panel_io_tx_param(io, (int)cmd, data_ptr, data_size);
  if (err != ESP_OK) {
    logger_.error("DSI write_command 0x{:02X} failed: {}", cmd, esp_err_to_name(err));
  }
}

void M5StackTab5::dsi_read_command(uint8_t cmd, std::span<uint8_t> data, uint32_t /*flags*/) {
  if (!lcd_handles_.io) {
    logger_.error("DSI read_command does not have a valid IO handle");
    return;
  }

  // logger_.debug("DSI read_command 0x{:02X} with {} bytes", cmd, length);

  esp_lcd_panel_io_handle_t io = lcd_handles_.io;
  void *data_ptr = data.data();
  size_t data_size = data.size();
  esp_err_t err = esp_lcd_panel_io_rx_param(io, (int)cmd, data_ptr, data_size);
  if (err != ESP_OK) {
    logger_.error("DSI read_command 0x{:02X} failed: {}", cmd, esp_err_to_name(err));
  }
}

} // namespace espp
