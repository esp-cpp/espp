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

#include <sdkconfig.h>

#include <driver/ppa.h>
#include <esp_heap_caps.h>
#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_ldo_regulator.h>

using namespace std::chrono_literals;

namespace espp {

// espp::DisplayRotation and lv_display_rotation_t enumerate the same four
// quarter-turns in the same order, and espp::Display itself converts between
// them by value (static_cast, see display.hpp). The rotation logic below leans
// on that correspondence — flush() keys off the LVGL enum while
// on_display_rotation() receives the espp one — so pin the mapping down
// explicitly and convert through these helpers only.
static_assert(static_cast<int>(DisplayRotation::LANDSCAPE) == LV_DISPLAY_ROTATION_0 &&
                  static_cast<int>(DisplayRotation::PORTRAIT) == LV_DISPLAY_ROTATION_90 &&
                  static_cast<int>(DisplayRotation::LANDSCAPE_INVERTED) ==
                      LV_DISPLAY_ROTATION_180 &&
                  static_cast<int>(DisplayRotation::PORTRAIT_INVERTED) == LV_DISPLAY_ROTATION_270,
              "espp::DisplayRotation and lv_display_rotation_t must stay value-compatible");
static constexpr lv_display_rotation_t to_lv_rotation(DisplayRotation rotation) {
  return static_cast<lv_display_rotation_t>(rotation);
}
static constexpr DisplayRotation to_display_rotation(lv_display_rotation_t rotation) {
  return static_cast<DisplayRotation>(rotation);
}

// The entire video path is RGB565-only: the DPI panel is configured for
// RGB565 in initialize_lcd(), flush()'s PPA rotation uses
// PPA_SRM_COLOR_MODE_RGB565 for both its input and output, and every buffer
// is sized in sizeof(Pixel) = 2-byte pixels. LVGL must therefore render
// RGB565 as well (LV_COLOR_DEPTH 16) — with any other color depth the flush
// callback would receive pixels these fixed color modes and sizes
// misinterpret (wrong colors at best, buffer overruns at worst). Refuse to
// build such a configuration instead of failing at runtime; supporting
// another depth means plumbing the active format through the DPI config, the
// PPA color modes, and the buffer sizing together.
static_assert(LV_COLOR_DEPTH == 16 && sizeof(M5StackTab5::Pixel) == sizeof(uint16_t),
              "The Tab5 video path (DPI panel config, PPA rotation color modes, buffer "
              "sizing) is RGB565-only; configure LVGL with LV_COLOR_DEPTH 16");

// Number of framebuffers the DPI panel is created with (esp_lcd_dpi_panel_config_t::num_fbs,
// used for every controller variant below). The direct-to-framebuffer PPA
// rotation in flush() caches THE single framebuffer pointer at init and writes
// into it unconditionally; with more than one framebuffer the driver flips
// which buffer is scanned out, and rotating into a fixed one would
// intermittently update a non-visible buffer. That optimization is therefore
// only implemented for exactly one framebuffer: raising this value requires
// first teaching the direct-to-framebuffer path to track the active
// framebuffer (or removing it in favor of the always-correct scratch-buffer
// path), and a static_assert in initialize_lcd() enforces that at compile
// time rather than letting the stale-pointer bug ship.
static constexpr uint8_t kNumDpiFramebuffers = 1;

// Alignment the PPA requires for its output buffer in external (PSRAM) memory:
// both the buffer pointer and the buffer size must be multiples of the data
// cache line size. On the ESP32-P4 external memory sits behind the L2 cache,
// whose line size is a Kconfig choice (CONFIG_CACHE_L2_CACHE_LINE_SIZE, 64 or
// 128 bytes) that cpu_start.c programs into the cache HAL at boot — so this
// public compile-time constant equals exactly the value the PPA driver itself
// validates against at runtime (ppa_check_buffer_alignment() checks
// s_platform.buf_alignment_size, which it obtains from that same cache HAL via
// the private esp_cache_get_alignment(MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA,
// ...)). Using the Kconfig constant keeps this file off the private
// esp_private/esp_cache_private.h header, which can move or break across IDF
// releases.
#if defined(CONFIG_CACHE_L2_CACHE_LINE_SIZE) && (CONFIG_CACHE_L2_CACHE_LINE_SIZE > 0)
static constexpr size_t kPpaOutBufferAlignment = CONFIG_CACHE_L2_CACHE_LINE_SIZE;
#else
// Fallback for IDF configurations that do not expose the symbol (it has
// shipped with ESP32-P4 support from the start, so this is belt-and-braces):
// use the largest L2 cache line the P4 supports so the alignment is never
// under-estimated — over-aligning is always safe here.
static constexpr size_t kPpaOutBufferAlignment = 128;
#endif

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
  bool st_acked = false;   // an ST device ACKed its address (even if reads failed)
  bool st_fw_read = false; // a firmware-version byte was successfully read
  int last_logged_fw = -1;
  for (int attempt = 0; attempt < 60; ++attempt) {
    const uint8_t fw_reg[2] = {0x00, 0x00};
    uint8_t fw_version = 0;
    // Quiet ACK probe first: while the ST touch engine is still booting it can
    // ACK-then-NACK a register read, which would log a scary (but harmless)
    // WriteRead error from the bus layer on every early attempt.
    if (i2c.probe_device(kStTouchAddress)) {
      st_acked = true;
      if (i2c.write_read(kStTouchAddress, fw_reg, sizeof(fw_reg), &fw_version, 1)) {
        st_fw_read = true;
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
      }
    } else if (i2c.probe_device(kGt911Address)) {
      // GT911 present -> original ILI9881 revision.
      return M5StackTab5::DisplayController::ILI9881;
    }
    std::this_thread::sleep_for(10ms);
  }

  if (st_acked) {
    // An ST TDDI is present but we could not identify it: either the firmware
    // version register read an unrecognized value, or the reads kept failing
    // for the whole poll window. Fall back to the ST7123 (the more common
    // part) rather than failing outright.
    if (st_fw_read) {
      logger_.warn("ST touch controller present but firmware version {:#04x} is unknown; "
                   "assuming ST7123",
                   last_logged_fw);
    } else {
      logger_.warn("ST touch controller ACKs but its firmware version could not be read; "
                   "assuming ST7123");
    }
    return M5StackTab5::DisplayController::ST7123;
  }

  // Unknown display controller
  return M5StackTab5::DisplayController::UNKNOWN;
}
bool M5StackTab5::initialize_lcd() {
  logger_.info("Initializing M5Stack Tab5 LCD (MIPI-DSI, {}x{})", display_width_, display_height_);

  // Re-initialization is NOT supported once the publication gate has opened:
  // clearing lcd_initialized_ here would not wait for readers that already
  // passed their acquire-load of true — a concurrent flush() or
  // write_lcd_lines() could then race the rebuild of lcd_handles_ /
  // display_driver_ / the framebuffer state it is still using. Rather than
  // add reader/writer synchronization to the hot flush path for a re-init
  // this BSP never needs (the panel hardware is fixed at boot), refuse.
  if (lcd_initialized_.load(std::memory_order_acquire)) {
    logger_.warn("LCD already initialized; re-initialization is not supported — skipping");
    return true;
  }

  // The publication gate is provably closed here: this is either the first
  // call or a retry after a failed attempt, and a failed attempt never
  // reaches the final store-release. No reader can therefore have observed
  // true, so the LCD state below (lcd_handles_, dpi_framebuffer_ +
  // dpi_framebuffer_bytes_, display_driver_, display_controller_) can be
  // built without racing them: even if the LVGL display already exists
  // (initialize_display() called first) and its thread is already pumping
  // flush(), flush()/write_lcd_lines()/on_display_rotation() all load-acquire
  // this flag and no-op while it is false, so none of them can observe a
  // partially initialized panel or a torn framebuffer-pointer/size pair. The
  // gate is store-released true as the final step of this function, after
  // every field has been written and the initial panel rotation applied.

  // Start every attempt from a clean slate for the state that is otherwise
  // only assigned on success paths below. A failed attempt never opened the
  // gate, so no reader has seen these, but it may still have left them set —
  // and a retry does not necessarily rewrite them (the framebuffer caching
  // below deliberately leaves them untouched when the query or alignment
  // check fails). Without this reset such a retry could open the gate with a
  // stale framebuffer pointer from the previous attempt still cached, and
  // flush()'s direct-to-framebuffer PPA path would then write through it.
  // (display_driver_ / display_controller_ need no reset here: display_driver_
  // is .reset() unconditionally below and both are rewritten together before
  // the gate can open.)
  dpi_framebuffer_ = nullptr;
  dpi_framebuffer_bytes_ = 0;

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
    // the ST7123: M5GFX's reference values are 900 vs 1040 Mbps, and this BSP
    // deliberately derates the ST7123 to 965 Mbps (margin for esp-lcd's PHY)
    // while using the M5GFX 900 Mbps for the ST7121. Driving an ST7121 at the
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
    dpi_cfg.num_fbs = kNumDpiFramebuffers;
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
    dpi_cfg.num_fbs = kNumDpiFramebuffers;
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
    dpi_cfg.num_fbs = kNumDpiFramebuffers;
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
    // The DPI pixel format is set here - in exactly one place, shared by all
    // controller variants - and is always RGB565: the whole video path is
    // RGB565-only (see the static_assert at the top of this file), and no
    // Kconfig or build flag selects any other panel format. The preprocessor
    // branch below only picks the IDF-version-specific field names for that
    // one format, never a different depth.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    dpi_cfg.in_color_format = LCD_COLOR_FMT_RGB565;
    dpi_cfg.out_color_format = LCD_COLOR_FMT_RGB565;
#else
    dpi_cfg.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
#endif
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

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
  // Use the 2D-DMA engine for esp_lcd_panel_draw_bitmap copies into the DPI
  // framebuffer. On ESP-IDF < 6.0 this was requested with the
  // esp_lcd_dpi_panel_config_t::flags.use_dma2d flag (set above); IDF 6.0
  // removed the flag in favor of this explicit call. Without it every
  // draw_bitmap is a CPU memcpy through the cache — for a full 720x1280 RGB565
  // frame that is ~1.8 MB read + ~1.8 MB written + a ~1.8 MB cache writeback
  // per flush, all PSRAM traffic that competes with the DPI panel's continuous
  // ~140 MB/s framebuffer scan-out DMA. Starving that scan-out DMA underruns
  // the DSI bridge FIFO, which shows up as streaks/tears along the panel's
  // scan-line axis (the driver logs "underrun happens" when it detects this).
  // The DMA2D copy runs without the CPU touching the data and completes via
  // the same on_color_trans_done callback registered above.
  //
  // Gate it per controller, though: DMA2D is a color-processing engine, not a
  // plain copy, and this repository documents (from hardware testing) that
  // routing draw_bitmap through it corrupts the RGB565 channel order on
  // ILI9881C-family DSI panels — colors render brighter/greener and alpha
  // blends come out wrong even though the framebuffer bytes are correct (see
  // components/esp32-p4-function-ev-board/src/video.cpp, ILI9881C/EK79007,
  // and components/esp32-p4-nano/src/video.cpp, which enables DMA2D only for
  // its JD9365 panel for the same reason). The Tab5's original revision uses
  // that same ILI9881 family, so it keeps the always-correct CPU copy path
  // and accepts the extra PSRAM traffic. The ST7121/ST7123 TDDI variants keep
  // DMA2D: no such corruption has been reported for them, they are the units
  // on which the underrun streaking this call addresses was reproduced, and
  // M5Stack's own Tab5 demo firmware ships with DMA2D enabled on production
  // units that are predominantly ST71xx.
  if (display_controller_ != DisplayController::ILI9881) {
    ret = esp_lcd_dpi_panel_enable_dma2d(lcd_handles_.panel);
    if (ret != ESP_OK) {
      // Not fatal: draw_bitmap falls back to the (slower) CPU copy.
      logger_.warn("Could not enable DMA2D for DPI draw_bitmap ({}); using CPU copies",
                   esp_err_to_name(ret));
    }
  } else {
    logger_.info("ILI9881 variant: keeping CPU draw_bitmap copies (DMA2D corrupts RGB565 "
                 "channel order on ILI9881C-family panels)");
  }
#endif

  // Cache the DPI framebuffer address so flush() can rotate directly into it
  // with the PPA (see flush()). The panel scans this buffer out continuously;
  // the espp draw/flush path itself never writes it with the CPU.
  //
  // This optimization is only valid with a single DPI framebuffer: caching one
  // fixed pointer assumes the panel scans that same buffer forever. With
  // num_fbs > 1 the driver flips between buffers and the PPA could rotate into
  // one that is not being scanned out. Multi-framebuffer operation is
  // intentionally unsupported until this path learns to track the active
  // framebuffer, and the static_assert below turns raising
  // kNumDpiFramebuffers without doing that work into a compile-time error
  // instead of an intermittent visual glitch.
  static_assert(kNumDpiFramebuffers == 1,
                "The direct-to-framebuffer PPA rotation caches a single framebuffer pointer; "
                "with multiple DPI framebuffers it must track the active one instead");
  {
    void *fb = nullptr;
    if (esp_lcd_dpi_panel_get_frame_buffer(lcd_handles_.panel, kNumDpiFramebuffers, &fb) ==
            ESP_OK &&
        fb != nullptr) {
      const size_t fb_bytes = static_cast<size_t>(display_width_) * display_height_ * sizeof(Pixel);
      // The PPA requires its output buffer pointer and size to be aligned to
      // the data cache line (see kPpaOutBufferAlignment). The esp_lcd driver
      // allocates the DPI framebuffer DMA-aligned, and 720*1280*2 is a
      // multiple of any such line size, but verify rather than assume — if it
      // does not hold, flush() simply keeps the scratch-buffer path.
      const size_t cache_align = kPpaOutBufferAlignment;
      // cppcheck-suppress [knownConditionTrueFalse, moduloofone] // cache_align
      // is a Kconfig compile-time constant (64 or 128); cppcheck's --force
      // explores a configuration where the macro folds to 1, trivializing the
      // checks. On real configurations both checks are meaningful.
      if ((reinterpret_cast<uintptr_t>(fb) % cache_align) == 0 && (fb_bytes % cache_align) == 0) {
        dpi_framebuffer_ = fb;
        dpi_framebuffer_bytes_ = fb_bytes;
      } else {
        logger_.warn("DPI framebuffer not cache-line aligned; PPA will rotate via scratch buffer");
      }
    } else {
      logger_.warn("Could not query the DPI framebuffer; PPA will rotate via scratch buffer");
    }
  }

  // Program the panel's initial scan direction BEFORE publishing the LCD
  // state, so the rotation decision flush() makes via
  // panel_handles_rotation() is valid from the very first frame that can
  // reach the panel. Ordering matters in the reversed init order
  // (initialize_display() first): an already-running LVGL thread may flush
  // the instant the gate opens, and if the gate opened before this call such
  // a flush could skip the PPA rotation (panel_handles_rotation() true) while
  // the panel's MADCTL still held the old scan direction. Applying the
  // rotation first closes that window. on_display_rotation() itself no-ops
  // while the gate is closed, so the init path calls the gate-free
  // apply_panel_rotation() helper directly — safe, because this is the same
  // thread that wrote display_driver_/display_controller_ above (no
  // synchronization needed against itself) and any concurrent flush() still
  // no-ops on the closed gate. Per init order:
  //  - Documented order (initialize_lcd() before initialize_display()):
  //    display_ is still null here, so no LVGL display or flush callback
  //    exists yet and espp::Display does not run an LVGL handler task of its
  //    own — nothing can race this call. It is also redundant-but-harmless in
  //    this order: the espp::Display constructor will call
  //    lv_display_set_rotation() with the initial rotation, which synchronously
  //    fires the LV_EVENT_RESOLUTION_CHANGED handler (in LVGL,
  //    update_resolution() sends the event as a direct call) and thus
  //    on_display_rotation() again before any flush can run.
  //  - Reversed order (initialize_display() first): any earlier rotation
  //    callback no-op'd on the closed gate (and display_driver_ did not exist
  //    yet), so (re)apply the current LVGL rotation now that the driver is up.
  //    Even if the application is already pumping LVGL on another thread, this
  //    cannot corrupt an in-flight flush: the MADCTL write travels on the DSI
  //    command channel, which never touches the DPI framebuffer or its DMA and
  //    is arbitrated against the video stream in hardware (see
  //    apply_panel_rotation()).
  // (With CONFIG_M5STACK_TAB5_ST7121_HW_ROTATION disabled — the default —
  // apply_panel_rotation() is a no-op and this call does nothing at all.)
  apply_panel_rotation(
      display_ ? to_display_rotation(lv_display_get_rotation(display_->get_lvgl_display()))
               : rotation);

  // Publish the fully initialized LCD state — the gate opens LAST. Every
  // field the cross-thread readers touch (lcd_handles_, dpi_framebuffer_ +
  // dpi_framebuffer_bytes_, display_driver_, display_controller_) has been
  // written above and the panel's initial MADCTL rotation has been applied,
  // so the release store here makes all of that visible to any
  // flush()/write_lcd_lines()/on_display_rotation() call that load-acquires
  // the flag as true. Until this store, those readers no-op (flush() just
  // signals lv_display_flush_ready()), which is what closes the
  // reversed-init-order races: with initialize_display() called first and the
  // application already pumping LVGL on another thread, a concurrent flush()
  // can no longer pass a plain lcd_handles_.panel null-check mid-build and
  // read a not-yet-init'd panel, a torn framebuffer-pointer/size pair, or a
  // panel whose scan direction does not yet match panel_handles_rotation().
  lcd_initialized_.store(true, std::memory_order_release);

  logger_.info("M5Stack Tab5 LCD initialization completed successfully");
  return true;
}

// Scratch buffer that holds the hardware-rotated frame produced by the PPA
// before it is handed to the panel (see flush()). Kept in PSRAM and aligned to
// the data-cache line size, which the PPA requires for its output buffer.
static M5StackTab5::Pixel *third_buffer = nullptr;
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
                                   // Forward LVGL rotation changes to the display driver so
                                   // panels that can rotate in hardware (ST7121) do so via
                                   // MADCTL instead of the PPA / software rotation in flush().
                                   .rotation_callback =
                                       std::bind_front(&M5StackTab5::on_display_rotation, this),
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
    // Throttle the PPA's AXI bursts (default PPA_DATA_BURST_LENGTH_128). The
    // DPI panel continuously scans its PSRAM framebuffer at ~140 MB/s; on the
    // ESP32-P4 a PPA client running full-length bursts against the same PSRAM
    // is known to starve that scan-out DMA and underrun the DSI bridge FIFO,
    // which appears as streaks along the panel's scan-line axis even with
    // 200 MHz hex PSRAM (see lvgl/lvgl#9590 - shorter PPA bursts fix the
    // artifacts at a small cost in PPA throughput; LVGL's own PPA integration
    // exposes the same knob as LV_PPA_BURST_LENGTH).
    ppa_cfg.data_burst_length = PPA_DATA_BURST_LENGTH_64;
    esp_err_t perr = ppa_register_client(&ppa_cfg, &g_ppa_client);
    if (perr != ESP_OK) {
      logger_.warn("Could not register PPA client ({}); rotation will fall back to software",
                   esp_err_to_name(perr));
      g_ppa_client = nullptr;
    }
  }
  // The PPA output buffer in external (PSRAM) memory must be aligned to the
  // data cache line size: both the pointer and the size must be a multiple of
  // it (see kPpaOutBufferAlignment).
  const size_t cache_align = kPpaOutBufferAlignment;
  size_t required_bytes = pixel_buffer_size * sizeof(Pixel);
  required_bytes = (required_bytes + cache_align - 1) / cache_align * cache_align;

  // Reuse the existing scratch buffer if it is already the right size; otherwise
  // free it first so a repeated initialize_display() call (e.g. re-init with a
  // different pixel buffer size) cannot leak the previous allocation.
  if (third_buffer == nullptr || third_buffer_bytes != required_bytes) {
    if (third_buffer != nullptr) {
      heap_caps_free(third_buffer);
      third_buffer = nullptr;
    }
    third_buffer_bytes = required_bytes;
    // Request MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA: the buffer is a PPA (DMA)
    // output target, and these are exactly the caps kPpaOutBufferAlignment is
    // derived for. The heap layer supports this combination for external
    // memory (esp_heap_adjust_alignment_to_hw() applies the cache-line
    // alignment/size the caps require, then maps the request onto the PSRAM
    // heap), so on the ESP32-P4 - whose PSRAM is DMA-capable - it yields a
    // DMA-usable PSRAM allocation.
    third_buffer = static_cast<Pixel *>(heap_caps_aligned_alloc(
        cache_align, third_buffer_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA));
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

bool M5StackTab5::panel_handles_rotation(lv_display_rotation_t rotation) const {
#if !CONFIG_M5STACK_TAB5_ST7121_HW_ROTATION
  // Panel-side rotation is DISABLED by default: hardware testing showed the
  // 180-degree MADCTL GS/SS scan flip renders corrupted on (at least some)
  // ST7121 units - the TDDI gate/source mux tables programmed at init (the
  // 0xAC block) are matched to the normal scan direction, and a MADCTL GS
  // flip alone reorders gate scanning without swapping them. All orientations
  // therefore use the PPA/flush-time rotation (the pre-existing, known-good
  // path). Enable CONFIG_M5STACK_TAB5_ST7121_HW_ROTATION to experiment.
  (void)rotation;
  return false;
#else
  // Only the ST7121 variant routes rotation to the panel (the ILI9881 and
  // ST7123 keep the historical PPA/flush-time rotation path). The Tab5 panels
  // are MIPI-DSI DPI (video mode) panels: the host streams a fixed 720x1280
  // raster, so the panel cannot swap axes for 90/270 (no MADCTL MV/GRAM
  // addressing in video mode) and those still need the frame rotated into the
  // framebuffer (PPA, or software fallback). The gate/source scan-direction
  // flips (MADCTL GS/SS) are applied by the panel itself through the espp
  // display driver's set_rotation() for 0 and 180 - EXPERIMENTAL, see the
  // Kconfig help.
  if (display_controller_ != DisplayController::ST7121) {
    return false;
  }
  return rotation == LV_DISPLAY_ROTATION_0 || rotation == LV_DISPLAY_ROTATION_180;
#endif
}

void M5StackTab5::on_display_rotation(const DisplayRotation &rotation) {
#if !CONFIG_M5STACK_TAB5_ST7121_HW_ROTATION
  // Panel-side rotation disabled (see panel_handles_rotation()): make this a
  // complete no-op so NO runtime MADCTL write ever reaches the panel - the
  // scan state stays exactly as the init sequence programmed it.
  (void)rotation;
#else
  // Acquire-load the publication gate (paired with the release store in
  // initialize_lcd()) before reading display_driver_ / display_controller_:
  // with initialize_display() called first, LVGL rotation events can arrive
  // while initialize_lcd() is still writing those fields. Bailing out here is
  // harmless — initialize_lcd() applies the current LVGL rotation via
  // apply_panel_rotation() before it opens the gate, so the panel state is
  // already consistent for the first flush() that observes the gate open.
  if (!lcd_initialized_.load(std::memory_order_acquire)) {
    return;
  }
  apply_panel_rotation(rotation);
#endif
}

void M5StackTab5::apply_panel_rotation(const DisplayRotation &rotation) {
#if !CONFIG_M5STACK_TAB5_ST7121_HW_ROTATION
  // Panel-side rotation disabled (see panel_handles_rotation()): complete
  // no-op so NO MADCTL write ever reaches the panel.
  (void)rotation;
#else
  if (!display_driver_ || display_controller_ != DisplayController::ST7121) {
    // Other variants keep the PPA/flush-time rotation path; nothing to do.
    return;
  }
  // Ordering: in its normal invocation — the espp::Display rotation callback,
  // via on_display_rotation() — this runs on the LVGL thread:
  // LV_EVENT_RESOLUTION_CHANGED is sent synchronously from inside
  // lv_display_set_rotation(), so it strictly precedes the
  // invalidation-driven flush() calls for the new orientation, and the MADCTL
  // state below is always consistent with the decision flush() makes via
  // panel_handles_rotation() (the same predicate, keyed on the same rotation
  // value via to_lv_rotation()). It is additionally called once directly from
  // initialize_lcd() (an init-thread call, not the LVGL thread, deliberately
  // BEFORE the lcd_initialized_ gate opens) to program the initial scan
  // direction; see the safety analysis at that call site. This helper is
  // gate-free: callers must guarantee display_driver_/display_controller_ are
  // safe to read (on_display_rotation() does so via its acquire load; the
  // init path wrote them on the same thread).
  //
  // Synchronization with the display pipeline: the MADCTL write goes out on
  // the DSI generic/DBI command channel (esp_lcd_panel_io_tx_param ->
  // mipi_dsi_hal_host_gen_write_dcs_command), which the DSI host peripheral
  // arbitrates against the DPI video stream in hardware. It never touches the
  // DPI framebuffer or its DMA, so it cannot corrupt a previous flush()'s
  // in-flight draw_bitmap copy. The panel may latch the new scan direction
  // mid scan-out, which can show as (at most) a single transient frame; that
  // is accepted here, since a rotation change is a full-screen visual
  // discontinuity anyway and LVGL follows it immediately with a full
  // invalidate/redraw of the new orientation.
  if (panel_handles_rotation(to_lv_rotation(rotation))) {
    // 0 / 180: the panel applies the rotation itself (scan-direction flip via
    // MADCTL); flush() writes unrotated buffers at unrotated coordinates.
    display_driver_->set_rotation(rotation);
  } else {
    // 90 / 270: a DPI video-mode panel cannot swap axes, so restore the
    // natural scan direction and let the PPA rotation in flush() do the full
    // transform.
    display_driver_->set_rotation(DisplayRotation::LANDSCAPE);
  }
#endif
}

void M5StackTab5::write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data,
                                  uint32_t user_data) {
  (void)user_data;
  // Acquire-load the publication gate (paired with the release store in
  // initialize_lcd()) instead of a plain lcd_handles_.panel null-check: this
  // runs on the caller's thread (e.g. the camera task) and must not race the
  // init thread's writes or draw into a panel that is not fully initialized.
  if (!lcd_initialized_.load(std::memory_order_acquire) || data == nullptr) {
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

  // Acquire-load the publication gate before touching any LCD state. This
  // pairs with the release store at the end of initialize_lcd(): observing
  // true guarantees lcd_handles_ (incl. a fully init'd panel, DMA2D-enabled
  // on the ST71xx variants), the dpi_framebuffer_/dpi_framebuffer_bytes_
  // pair, display_driver_ and display_controller_ are all completely written
  // and will not be written again (the gate never closes once open —
  // initialize_lcd() refuses to re-run). Until then (LCD not yet initialized)
  // just tell LVGL the flush is done and drop the frame.
  if (!lcd_initialized_.load(std::memory_order_acquire)) {
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
  // When the panel itself applies the rotation (ST7121, 180 degrees via the
  // display driver's MADCTL scan flip - see on_display_rotation()), the
  // logical frame is written to the framebuffer unrotated and at unrotated
  // coordinates; the panel flips the whole frame at scan-out, which lands each
  // partial area exactly where LVGL's rotated mapping expects it.
  if (rotation > LV_DISPLAY_ROTATION_0 && !panel_handles_rotation(rotation) &&
      third_buffer != nullptr) {
    int32_t ww = lv_area_get_width(area);
    int32_t hh = lv_area_get_height(area);
    lv_color_format_t cf = lv_display_get_color_format(disp);
    // Map the logical (LVGL) area to physical panel coordinates up front: the
    // direct-to-framebuffer PPA path needs the rotated destination offsets
    // before the PPA runs, and the fallback paths need them for draw_bitmap.
    // Rotate a local copy — LVGL handed us a const pointer, so do not mutate
    // its area in place.
    lv_area_t rotated_area = *area;
    lv_display_rotate_area(disp, &rotated_area);
    offsetx1 = rotated_area.x1;
    offsetx2 = rotated_area.x2;
    offsety1 = rotated_area.y1;
    offsety2 = rotated_area.y2;
    if (g_ppa_client != nullptr && dpi_framebuffer_ != nullptr) {
      // Hardware rotation via the PPA, writing the rotated block DIRECTLY into
      // the DPI panel's framebuffer at the rotated offset (the PPA output
      // supports placing a block inside a larger picture). This halves the
      // PSRAM traffic of the previous scratch-buffer approach (PPA write +
      // draw_bitmap read + write), which matters because the DSI scan-out DMA
      // is reading the same PSRAM continuously and underruns - visible as
      // streaks - when the flush path hogs the bandwidth.
      //
      // The LVGL rotation maps directly onto the PPA rotation angle (LVGL 90
      // -> PPA 90, 180 -> 180, 270 -> 270); this is the mapping verified on
      // hardware. If a different panel comes out turned the wrong way, swap
      // the 90 and 270 cases here.
      ppa_srm_rotation_angle_t angle = PPA_SRM_ROTATION_ANGLE_0;
      if (rotation == LV_DISPLAY_ROTATION_90) {
        angle = PPA_SRM_ROTATION_ANGLE_90;
      } else if (rotation == LV_DISPLAY_ROTATION_180) {
        angle = PPA_SRM_ROTATION_ANGLE_180;
      } else if (rotation == LV_DISPLAY_ROTATION_270) {
        angle = PPA_SRM_ROTATION_ANGLE_270;
      }
      ppa_srm_oper_config_t srm = {};
      srm.in.buffer = px_map;
      srm.in.pic_w = ww;
      srm.in.pic_h = hh;
      srm.in.block_w = ww;
      srm.in.block_h = hh;
      srm.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
      srm.out.buffer = dpi_framebuffer_;
      srm.out.buffer_size = dpi_framebuffer_bytes_;
      srm.out.pic_w = display_width_;
      srm.out.pic_h = display_height_;
      srm.out.block_offset_x = offsetx1;
      srm.out.block_offset_y = offsety1;
      srm.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
      srm.rotation_angle = angle;
      srm.scale_x = 1.0f;
      srm.scale_y = 1.0f;
      srm.mode = PPA_TRANS_MODE_BLOCKING;
      if (ppa_do_scale_rotate_mirror(g_ppa_client, &srm) == ESP_OK) {
        // The rotated pixels are already in the scanned-out framebuffer and
        // the PPA driver performed the cache maintenance (write-back of the
        // source window, invalidate of the destination window). The espp
        // flush path never dirties the framebuffer with the CPU (draw_bitmap
        // copies run on the DMA2D engine on the ST71xx variants, and the CPU
        // copy path — used by the ILI9881 variant and as the DMA2D fallback —
        // writes back the cache before returning), so there is nothing left
        // to sync and
        // no draw_bitmap call is needed: signal LVGL directly.
        lv_display_flush_ready(disp);
        return;
      }
      // On failure fall through to the scratch-buffer software rotation: the
      // framebuffer may hold a partial block, but the fallback redraws the
      // full area at the same destination.
    }
    {
      // Fallback: the PPA client failed to register, the framebuffer could
      // not be queried, or the PPA operation failed. Rotate on the CPU into
      // the scratch buffer, fully overwriting it, and hand it to draw_bitmap
      // below.
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
