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

// Number of frame buffers the DPI panel is created with (num_fbs, the same for
// every controller variant below). More than one lets an application render
// straight into them and switch whole frames (see frame_buffers()).
static constexpr uint8_t kNumDpiFramebuffers = CONFIG_M5STACK_TAB5_DPI_NUM_FBS;
static_assert(kNumDpiFramebuffers >= 1 && kNumDpiFramebuffers <= M5StackTab5::max_frame_buffers,
              "CONFIG_M5STACK_TAB5_DPI_NUM_FBS must be 1..3 (the DPI panel driver's limit)");

// Whether this ESP-IDF's DPI panel raises its frame-done event from the DMA
// transfer-done interrupt after re-arming on the queued frame buffer
// (on_frame_buf_complete: v5.5.5+, v6.0.3+). Earlier versions raise
// on_refresh_done from the DSI bridge's VSYNC interrupt on chips that have
// one (the P4), which is not synchronized with the re-arm; present_frame()
// then waits for two events. The DMA2D / frame-buffer draw_bitmap ordering
// bug of v6.0.0 is handled separately in initialize_lcd().
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 3) ||                                             \
    (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 5) &&                                            \
     ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0))
static constexpr bool kFrameDoneIsBufferComplete = true;
#else
static constexpr bool kFrameDoneIsBufferComplete = false;
#endif

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
  dpi_framebuffer_.store(nullptr, std::memory_order_release);
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

  // Keep the panel's frame buffers for frame_buffers() and for
  // DisplayConfig::render_to_frame_buffers. (The driver reads only the first
  // kNumDpiFramebuffers of the pointers passed.)
  num_frame_buffers_ = 0;
  frame_buffers_ = {};
  ret = esp_lcd_dpi_panel_get_frame_buffer(lcd_handles_.panel, kNumDpiFramebuffers,
                                           &frame_buffers_[0], &frame_buffers_[1],
                                           &frame_buffers_[2]);
  if (ret != ESP_OK) {
    logger_.error("Failed to get the DPI panel frame buffers: {}", esp_err_to_name(ret));
    frame_buffers_ = {};
    return false;
  }
  num_frame_buffers_ = kNumDpiFramebuffers;

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
  esp_lcd_dpi_panel_event_callbacks_t cbs = {};
  cbs.on_color_trans_done = &M5StackTab5::notify_lvgl_flush_ready;
  // Completion semaphore for the serialized synchronous draws (see
  // draw_and_wait()); created before the callback that gives it can fire.
  if (draw_done_sem_ == nullptr) {
    draw_done_sem_ = xSemaphoreCreateBinary();
    if (draw_done_sem_ == nullptr) {
      logger_.error("Failed to create the draw-completion semaphore");
      return false;
    }
  }
  if constexpr (kNumDpiFramebuffers > 1) {
    // Frame-done event for present_frame(): the DMA has finished a frame and
    // restarted on the frame buffer queued last (cur_fb_index), i.e. a queued
    // switch has happened. ESP-IDF v5.5.5 / v6.0.3 / v6.1 renamed this
    // callback from on_refresh_done (now deprecated) to on_frame_buf_complete
    // and added a separate on_vsync; pick whichever this ESP-IDF has.
    auto set_frame_done_callback = [](auto &callbacks, auto callback) {
      if constexpr (requires { callbacks.on_frame_buf_complete; }) {
        callbacks.on_frame_buf_complete = callback;
      } else {
        callbacks.on_refresh_done = callback;
      }
    };
    // Created before the callback is registered, so the ISR never sees null.
    if (frame_done_sem_ == nullptr) {
      frame_done_sem_ = xSemaphoreCreateBinary();
    }
    if (frame_done_sem_ != nullptr) {
      set_frame_done_callback(cbs, &M5StackTab5::notify_frame_done);
    } else {
      logger_.warn("Could not create the frame-done semaphore; present_frame() will not wait");
    }
  }
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
  //
  // ESP-IDF v6.0.0 only: its DPI draw_bitmap() checks the DMA2D hook before the
  // "pointer is inside a frame buffer" test (fixed in v6.0.1), so with DMA2D on
  // and 2+ frame buffers a frame-buffer switch would become a full-frame copy
  // into the buffer on screen. Keep the CPU path there.
#if ESP_IDF_VERSION == ESP_IDF_VERSION_VAL(6, 0, 0)
  constexpr bool dma2d_breaks_fb_switch = kNumDpiFramebuffers > 1;
#else
  constexpr bool dma2d_breaks_fb_switch = false;
#endif
  if constexpr (dma2d_breaks_fb_switch) {
    logger_.info("ESP-IDF v6.0.0 with {} frame buffers: keeping CPU draw_bitmap copies (DMA2D "
                 "would turn a frame-buffer switch into a copy)",
                 kNumDpiFramebuffers);
  } else if (display_controller_ != DisplayController::ILI9881) {
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
  // With several DPI framebuffers the driver scans whichever one was last
  // handed to draw_bitmap(): nothing switches buffers except present_frame(),
  // which keeps dpi_framebuffer_ pointing at the buffer it switched to, so the
  // PPA rotation always targets the scanned-out buffer whatever the count.
  // (An application that presents its own frames does not use the LVGL
  // partial-mode flush at the same time, and DIRECT mode never rotates.)
  {
    void *fb = frame_buffers_[0];
    if (fb != nullptr) {
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
        dpi_framebuffer_.store(fb, std::memory_order_release);
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
  // Use the configured initial `rotation` rather than querying LVGL here:
  // in the reversed init order the LVGL update task may already be running,
  // and lv_display_get_rotation() from this (init) thread would read LVGL
  // state without the application's LVGL lock (LV_USE_OS == LV_OS_NONE). Any
  // rotation the app sets later is applied by on_display_rotation() (the LVGL
  // rotation callback, which runs on the GUI task), so the panel still tracks
  // runtime changes; this call only establishes the initial MADCTL.
  // ... except that a rotation the application set before this point (with
  // initialize_display() first) was recorded by on_display_rotation() in
  // requested_rotation_ rather than dropped, so apply the latest request, not
  // the compile-time default. Sequentially consistent with the store/load pair
  // in on_display_rotation(): either this load sees that request, or that
  // thread's gate load sees the gate open and applies it itself.
  apply_panel_rotation(requested_rotation_.load(std::memory_order_seq_cst));

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
  lcd_initialized_.store(true, std::memory_order_seq_cst);

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
  return initialize_display(DisplayConfig{.pixel_buffer_size = pixel_buffer_size});
}

bool M5StackTab5::initialize_display(const DisplayConfig &config) {
  if (display_ && direct_render_ != config.render_to_frame_buffers) {
    logger_.error("The LVGL display is already initialized {} the frame buffers",
                  direct_render_ ? "on" : "without");
    return false;
  }
  const Display<Pixel>::LvglConfig lvgl_config{
      .width = display_width_,
      .height = display_height_,
      .flush_callback = std::bind_front(&M5StackTab5::flush, this),
      // Forward LVGL rotation changes to the display driver so panels that can
      // rotate in hardware (ST7121) do so via MADCTL instead of the PPA /
      // software rotation in flush().
      .rotation_callback = std::bind_front(&M5StackTab5::on_display_rotation, this),
      .rotation = rotation};
  const Display<Pixel>::OledConfig oled_config{
      .set_brightness_callback =
          [this](float brightness) { this->brightness(brightness * 100.0f); },
      .get_brightness_callback = [this]() { return this->brightness() / 100.0f; }};

  if (config.render_to_frame_buffers) {
    if (num_frame_buffers_ < 2) {
      logger_.error("render_to_frame_buffers needs initialize_lcd() first and "
                    "CONFIG_M5STACK_TAB5_DPI_NUM_FBS >= 2 ({} frame buffers)",
                    num_frame_buffers_);
      return false;
    }
    // DIRECT mode renders straight into the panel's frame buffers, whose rows
    // are exactly display_width_ pixels apart. LVGL lays its draw buffers out
    // with its own stride rule (LV_DRAW_BUF_STRIDE_ALIGN): 720 RGB565 pixels
    // are 1440 bytes, a multiple of neither 64 nor 128, so a raised stride
    // alignment would silently skew every row. Refuse rather than draw skewed.
    const uint32_t lvgl_stride =
        lv_draw_buf_width_to_stride(display_width_, LV_COLOR_FORMAT_RGB565);
    if (lvgl_stride != display_width_ * sizeof(Pixel)) {
      logger_.error("render_to_frame_buffers needs LVGL's row stride ({} bytes) to equal the "
                    "panel's ({} bytes); set LV_DRAW_BUF_STRIDE_ALIGN so it does",
                    lvgl_stride, display_width_ * sizeof(Pixel));
      return false;
    }
    // LVGL cannot rotate in DIRECT mode (it renders in place, and the PPA /
    // software rotation of flush() has no separate draw buffer to rotate
    // from), so the display must come up unrotated and stay so.
    if (rotation != DisplayRotation::LANDSCAPE ||
        requested_rotation_.load(std::memory_order_seq_cst) != DisplayRotation::LANDSCAPE) {
      logger_.error("render_to_frame_buffers needs the display rotation to be 0 (LANDSCAPE); "
                    "LVGL cannot rotate in DIRECT mode");
      return false;
    }
    logger_.info("Initializing LVGL display in DIRECT mode on the DPI frame buffers");
    if (!display_) {
      // LVGL renders its first frame into vram0. The panel scans frame buffer 0
      // out from the start, so hand LVGL buffer 1 first: the first frame is
      // drawn off screen like every later one. (With 3 frame buffers LVGL uses
      // buffers 1 and 0.)
      display_ = std::make_shared<Display<Pixel>>(
          lvgl_config, oled_config,
          Display<Pixel>::StaticMemoryConfig{
              .pixel_buffer_size = display_width_ * display_height_,
              .vram0 = static_cast<Pixel *>(frame_buffers_[1]),
              .vram1 = static_cast<Pixel *>(frame_buffers_[0]),
          },
          Logger::Verbosity::WARN);
      // espp::Display sets the buffers up for PARTIAL rendering; render whole
      // frames in place instead (nothing has been rendered yet).
      lv_display_set_render_mode(display_->get_lvgl_display(), LV_DISPLAY_RENDER_MODE_DIRECT);
      direct_render_ = true;
    }
    // No PPA client and no rotation scratch buffer: flush() does not rotate in
    // DIRECT mode, and LVGL's buffers are the frame buffers themselves.
    logger_.info("LVGL display initialized");
    return true;
  }

  const size_t pixel_buffer_size = config.pixel_buffer_size;
  logger_.info("Initializing LVGL display with pixel buffer size: {} pixels", pixel_buffer_size);
  if (!display_) {
    display_ = std::make_shared<Display<Pixel>>(
        lvgl_config, oled_config,
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
    } else {
      // flush() queues its rotations non-blocking; this callback completes
      // them (see notify_ppa_trans_done()). Without it the PPA path cannot be
      // used, since nothing would ever signal LVGL.
      ppa_event_callbacks_t ppa_cbs = {};
      ppa_cbs.on_trans_done = &M5StackTab5::notify_ppa_trans_done;
      perr = ppa_client_register_event_callbacks(g_ppa_client, &ppa_cbs);
      if (perr != ESP_OK) {
        logger_.warn("Could not register the PPA completion callback ({}); rotation will fall "
                     "back to software",
                     esp_err_to_name(perr));
        ppa_unregister_client(g_ppa_client);
        g_ppa_client = nullptr;
      }
    }
  }
  // The software-rotation scratch buffer (see ensure_rotation_scratch_buffer())
  // is the size of the LVGL draw buffer, aligned to the data cache line like a
  // PPA output buffer (see kPpaOutBufferAlignment). It is NOT allocated here:
  // with the PPA rotating straight into the panel framebuffer it is only ever
  // used by the software fallback, so it is allocated the first time that
  // fallback runs (never, on a board whose PPA works) - or never, with
  // CONFIG_M5STACK_TAB5_ROTATION_SOFTWARE_FALLBACK disabled.
  const size_t cache_align = kPpaOutBufferAlignment;
  size_t required_bytes = pixel_buffer_size * sizeof(Pixel);
  required_bytes = (required_bytes + cache_align - 1) / cache_align * cache_align;
  rotation_scratch_bytes_ = required_bytes;
  // A buffer left from an earlier initialize_display() with another draw
  // buffer size is the wrong size: free it so the next fallback reallocates.
  if (third_buffer != nullptr && third_buffer_bytes != required_bytes) {
    heap_caps_free(third_buffer);
    third_buffer = nullptr;
    third_buffer_bytes = 0;
  }

  logger_.info("LVGL display initialized");
  return true;
}

size_t M5StackTab5::rotated_display_width() const {
  auto rotation =
      lv_display_get_rotation(display_ ? display_->get_lvgl_display() : lv_display_get_default());
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
  auto rotation =
      lv_display_get_rotation(display_ ? display_->get_lvgl_display() : lv_display_get_default());
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
  // Record the latest request before the gate check, whatever the build
  // variant: with initialize_display() called first, a rotation the
  // application sets before initialize_lcd() would otherwise be dropped here
  // and never re-sent, leaving the panel at the compile-time default while
  // flush() believes the panel handles the rotation. initialize_lcd() applies
  // this value when it brings the panel up (see the ordering note there).
  requested_rotation_.store(rotation, std::memory_order_seq_cst);
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
  if (!lcd_initialized_.load(std::memory_order_seq_cst)) {
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

bool M5StackTab5::ensure_rotation_scratch_buffer() {
#if !CONFIG_M5STACK_TAB5_ROTATION_SOFTWARE_FALLBACK
  // PPA-only rotation: the scratch buffer is never allocated.
  return false;
#else
  if (third_buffer != nullptr) {
    return true;
  }
  if (rotation_scratch_bytes_ == 0) {
    return false; // initialize_display() has not run
  }
  // Request MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA with the cache-line alignment a
  // PPA output buffer needs (kPpaOutBufferAlignment): the heap layer maps the
  // request onto the PSRAM heap, and on the ESP32-P4 - whose PSRAM is
  // DMA-capable - yields a DMA-usable PSRAM allocation.
  third_buffer = static_cast<Pixel *>(heap_caps_aligned_alloc(
      kPpaOutBufferAlignment, rotation_scratch_bytes_, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA));
  if (third_buffer == nullptr) {
    logger_.error_rate_limited("Could not allocate the rotation scratch buffer ({} bytes)",
                               rotation_scratch_bytes_);
    return false;
  }
  third_buffer_bytes = rotation_scratch_bytes_;
  logger_.info("Rotating in software: allocated the {} byte rotation scratch buffer",
               rotation_scratch_bytes_);
  return true;
#endif
}

// How long draw_and_wait() waits for the DPI driver's transfer-done callback.
// A DMA2D copy of a full 720x1280 RGB565 frame takes a few milliseconds even
// under PSRAM contention, so anything near this is a wedged transfer.
static constexpr uint32_t kDrawTimeoutMs = 200;

bool M5StackTab5::draw_and_wait(int x1, int y1, int x2, int y2, const void *data) {
  // Caller holds panel_op_mutex_ and has called wait_for_transfer_idle(): no
  // transfer is in flight and the completion semaphore holds no token. Issue
  // the draw, then block until on_color_trans_done gives the token for it.
  transfer_in_flight_.store(true, std::memory_order_release);
  esp_err_t err = esp_lcd_panel_draw_bitmap(lcd_handles_.panel, x1, y1, x2, y2, data);
  if (err != ESP_OK) {
    // Rejected or failed - no completion callback will arrive.
    transfer_in_flight_.store(false, std::memory_order_release);
    logger_.warn_rate_limited("draw_bitmap failed: {}", esp_err_to_name(err));
    return false;
  }
  if (draw_done_sem_ == nullptr) {
    transfer_in_flight_.store(false, std::memory_order_release);
    return true;
  }
  // Bounded wait so a missing completion cannot wedge the caller forever. A
  // timeout is a failure, not a success: the copy may still be reading the
  // caller's buffer, and the caller is about to hand that buffer back. The
  // transfer stays in flight - its token is still owed - so the next panel
  // operation waits for it again instead of attributing the late completion
  // to its own transfer (see wait_for_transfer_idle()).
  if (xSemaphoreTake(draw_done_sem_, pdMS_TO_TICKS(kDrawTimeoutMs)) != pdTRUE) {
    logger_.warn_rate_limited("draw_bitmap ({},{})-({},{}) did not complete within {} ms", x1, y1,
                              x2, y2, kDrawTimeoutMs);
    return false;
  }
  transfer_in_flight_.store(false, std::memory_order_release);
  return true;
}

bool M5StackTab5::wait_for_transfer_idle() {
  // Caller holds panel_op_mutex_. Token accounting: every transfer that sets
  // transfer_in_flight_ is completed by exactly one token on draw_done_sem_,
  // given by the completion ISR, and whoever consumes the token clears the
  // flag - here, or draw_and_wait() for its own transfer. The ISR never
  // clears the flag itself, so a transfer stays in flight until its token has
  // been consumed under this mutex, and a token can never be attributed to a
  // later transfer, whichever core the ISR runs on.
  //
  // An LVGL flush leaves its draw in flight (see flush()); nothing else may be
  // issued, and no engine may write the framebuffer, until it is complete.
  if (!transfer_in_flight_.load(std::memory_order_acquire)) {
    return true;
  }
  if (draw_done_sem_ == nullptr) {
    transfer_in_flight_.store(false, std::memory_order_release);
    return true;
  }
  if (xSemaphoreTake(draw_done_sem_, pdMS_TO_TICKS(kDrawTimeoutMs)) != pdTRUE) {
    // A wedged transfer. It stays in flight (the caller must not issue its
    // own operation on top of it) and LVGL, if it is the one waiting, stays
    // waiting: the ISR releases it when the completion finally comes. The
    // display stalls visibly rather than two engines working the same memory.
    logger_.warn_rate_limited("draw_bitmap did not complete within {} ms", kDrawTimeoutMs);
    return false;
  }
  transfer_in_flight_.store(false, std::memory_order_release);
  return true;
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
  std::lock_guard<std::mutex> lock(panel_op_mutex_);
  // An LVGL flush's copy may still be in flight; it must finish before this
  // draw is issued (single-flight) and before its completion is waited for.
  if (!wait_for_transfer_idle()) {
    return; // the previous transfer is wedged; do not stack another on it
  }
  draw_and_wait(xs, ys, xe + 1, ye + 1, data);
}

bool M5StackTab5::present_frame(const void *frame_buffer, uint32_t timeout_ms) {
  // frame_buffers() is empty until the lcd_initialized_ gate is open
  // (acquire), so this also refuses a panel that is not fully initialized.
  // Any pointer inside a frame buffer selects that buffer, as the driver's
  // draw_bitmap does.
  const auto fbs = frame_buffers();
  const auto *p = static_cast<const uint8_t *>(frame_buffer);
  const auto owner = std::find_if(fbs.begin(), fbs.end(), [p](void *fb) {
    const auto *base = static_cast<const uint8_t *>(fb);
    return base != nullptr && p >= base && p < base + frame_buffer_size();
  });
  if (frame_buffer == nullptr || owner == fbs.end()) {
    logger_.error("present_frame() needs a pointer inside one of the panel's frame buffers");
    return false;
  }
  // One presenter at a time: the frame-done semaphore has a single token and
  // a single waiter per switch, so a second caller must not steal this one's
  // event. (The LVGL flush is the only presenter in DIRECT mode; this guards
  // an application that drives the frame buffers itself from another task.)
  std::lock_guard<std::mutex> present_lock(present_mutex_);
  {
    // Queue the switch: for a frame buffer the driver writes the cache back,
    // makes it the next one to scan out (cur_fb_index) and calls
    // on_color_trans_done synchronously, so draw_and_wait() returns at once.
    // The DMA takes the new buffer up when it restarts at the end of the
    // frame it is sending. Serialized with write_lcd_lines() like every other
    // panel draw; the frame-done wait below is outside the mutex, so a direct
    // write is not held up for a refresh.
    std::lock_guard<std::mutex> lock(panel_op_mutex_);
    // Like every panel operation: wait for the transfer in flight (an
    // asynchronous LVGL flush copy or PPA rotation) before issuing one, so
    // the token accounting in wait_for_transfer_idle() / draw_and_wait()
    // stays exact. The driver completes a frame-buffer draw synchronously
    // through on_color_trans_done, which gives the token from task context.
    if (!wait_for_transfer_idle()) {
      logger_.error("present_frame(): a previous transfer is wedged; not switching");
      return false;
    }
    if (!draw_and_wait(0, 0, display_width_, display_height_, frame_buffer)) {
      logger_.error("present_frame(): draw_bitmap failed");
      return false;
    }
    // The panel now scans (or is about to scan) this buffer: point the PPA
    // rotation path at it (see initialize_lcd()). Only if that path was
    // enabled at init, i.e. the buffers passed its alignment check.
    if (dpi_framebuffer_.load(std::memory_order_relaxed) != nullptr) {
      dpi_framebuffer_.store(*owner, std::memory_order_release);
    }
  }
  if (!vsync_enabled_.load(std::memory_order_relaxed) || frame_done_sem_ == nullptr) {
    return true;
  }
  // Drop a frame-done event that came before this point: it may have
  // restarted the DMA on the previous buffer. Any later one restarts it on
  // the new buffer. (If the switch itself landed between draw_bitmap() and
  // here, this waits one refresh longer, which is safe.)
  xSemaphoreTake(frame_done_sem_, 0);
  // How many events prove the switch happened. With on_frame_buf_complete
  // (ESP-IDF v5.5.5 / v6.0.3 and later) the driver raises the event from the
  // DMA transfer-done interrupt right after it re-armed on cur_fb_index, so
  // one event after the draw is the switch. Before that the P4 raises
  // on_refresh_done from the DSI bridge's VSYNC interrupt, and the DMA fetch
  // of a frame completes a porch BEFORE that frame's VSYNC: the first VSYNC
  // after the draw may belong to a frame whose re-arm preceded the draw
  // (still the old buffer). The DMA re-arms exactly once between two
  // consecutive VSYNCs, so the second VSYNC after the draw is past a re-arm
  // that saw the new index; one refresh of extra latency on those versions.
  constexpr int kFrameDoneEventsForSwitch = kFrameDoneIsBufferComplete ? 1 : 2;
  bool switched = true;
  for (int i = 0; i < kFrameDoneEventsForSwitch; ++i) {
    if (xSemaphoreTake(frame_done_sem_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
      switched = false;
      break;
    }
  }
  if (switched) {
    return true;
  }
  // A panel that stopped refreshing must not freeze the caller: the switch
  // stays queued and happens whenever the DMA restarts.
  if (!vsync_timeout_logged_.exchange(true, std::memory_order_relaxed)) {
    logger_.warn("present_frame(): no frame-done event within {} ms (logged once)", timeout_ms);
  }
  return false;
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
  // (the PPA rotation, waiting for a previous transfer) is therefore safe here.
  // The draw_bitmap copy itself is NOT waited for: it is left in flight and the
  // ISR signals LVGL when it completes, so LVGL renders the next area into its
  // other draw buffer while this one is being copied (see transfer_in_flight_).

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

  // Use the display actually being flushed, not the global default display.
  auto rotation = lv_display_get_rotation(disp);
  // Cache the rotation for the camera task, which must not call LVGL from its
  // own thread. flush() runs on the LVGL (GUI) thread, so reading it here is
  // safe; the camera task reads the cached atomic instead.
  camera_display_rotation_.store(static_cast<uint8_t>(rotation), std::memory_order_relaxed);

  if (direct_render_) {
    // DIRECT mode (DisplayConfig::render_to_frame_buffers): LVGL has rendered
    // the dirty areas in place, into the frame buffer px_map points to, and
    // calls this once per area. When the last area is done, present_frame()
    // switches the panel to that buffer (no copy). LVGL then renders the next
    // frame into the other buffer.
    static bool rotation_warned = false; // flush() only runs on the LVGL task
    if (rotation != LV_DISPLAY_ROTATION_0 && !rotation_warned) {
      logger_.warn("Display rotation is not supported when rendering into the frame buffers");
      rotation_warned = true;
    }
    if (lv_display_flush_is_last(disp)) {
      // Waits until the panel scans px_map out, so LVGL never renders the
      // next frame into the buffer the panel is still reading (see
      // present_frame()). Two costs are inherent to DIRECT mode with two
      // buffers and deliberately not optimized here: LVGL copies the previous
      // frame's dirty areas between the buffers on the CPU before each render
      // (refr_sync_areas), and the switch writes the whole frame's cache back
      // - the sync copy dirties lines outside this frame's areas, so a
      // per-area write-back would have to union the previous frame's areas.
      // This flush never leaves a transfer in flight (lvgl_flush_pending_ is
      // not set): the switch completes synchronously and LVGL is released
      // right here.
      present_frame(px_map);
    }
    lv_display_flush_ready(disp);
    return;
  }

  // When the panel itself applies the rotation (ST7121, 180 degrees via the
  // display driver's MADCTL scan flip - see on_display_rotation()), the
  // logical frame is written to the framebuffer unrotated and at unrotated
  // coordinates; the panel flips the whole frame at scan-out, which lands each
  // partial area exactly where LVGL's rotated mapping expects it.
  if (rotation > LV_DISPLAY_ROTATION_0 && !panel_handles_rotation(rotation)) {
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
    // The scanned-out framebuffer (present_frame() moves it when it switches
    // buffers; see initialize_lcd()).
    void *const target_framebuffer = dpi_framebuffer_.load(std::memory_order_acquire);
    if (g_ppa_client != nullptr && target_framebuffer != nullptr) {
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
      srm.out.buffer = target_framebuffer;
      srm.out.buffer_size = dpi_framebuffer_bytes_;
      srm.out.pic_w = display_width_;
      srm.out.pic_h = display_height_;
      srm.out.block_offset_x = offsetx1;
      srm.out.block_offset_y = offsety1;
      srm.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
      srm.rotation_angle = angle;
      srm.scale_x = 1.0f;
      srm.scale_y = 1.0f;
      // Non-blocking, like the DMA2D copy of the unrotated path: the PPA
      // driver writes back the source window and invalidates the destination
      // window when the operation is queued, the operation is left in flight
      // and notify_ppa_trans_done() signals LVGL when it finishes, so LVGL
      // renders the next area into its other draw buffer meanwhile. The next
      // panel operation waits for it first (wait_for_transfer_idle()).
      srm.mode = PPA_TRANS_MODE_NON_BLOCKING;
      srm.user_data = this;
      // Serialize the framebuffer write with draw_bitmap (write_lcd_lines /
      // the non-PPA flush tail) so two engines never write the panel FB at once.
      esp_err_t ppa_err;
      {
        std::lock_guard<std::mutex> lock(panel_op_mutex_);
        if (!wait_for_transfer_idle()) {
          // a direct write's transfer is wedged: drop this area rather than
          // rotate on top of it, and keep LVGL alive
          lv_display_flush_ready(disp);
          return;
        }
        // Publish the display pointer before the pending flag (release), so
        // the ISR's acquire on the flag always sees the right display.
        lvgl_flush_disp_ = disp;
        lvgl_flush_pending_.store(true, std::memory_order_release);
        transfer_in_flight_.store(true, std::memory_order_release);
        ppa_err = ppa_do_scale_rotate_mirror(g_ppa_client, &srm);
        if (ppa_err != ESP_OK) {
          // Not queued: no completion will arrive.
          transfer_in_flight_.store(false, std::memory_order_release);
          lvgl_flush_pending_.store(false, std::memory_order_relaxed);
        }
      }
      if (ppa_err == ESP_OK) {
        // The rotated pixels land in the scanned-out framebuffer with the
        // cache maintenance done by the PPA driver; the espp flush path never
        // dirties the framebuffer with the CPU (draw_bitmap copies run on the
        // DMA2D engine on the ST71xx variants, and the CPU copy path — used by
        // the ILI9881 variant and as the DMA2D fallback — writes back the
        // cache before returning), so no draw_bitmap call is needed and the
        // PPA completion callback signals LVGL.
        return;
      }
      logger_.warn_rate_limited("PPA rotation failed ({}); rotating in software",
                                esp_err_to_name(ppa_err));
      // On failure fall through to the scratch-buffer software rotation: the
      // framebuffer may hold a partial block, but the fallback redraws the
      // full area at the same destination.
    }
    {
      // Fallback: the PPA client failed to register, the framebuffer could
      // not be queried, or the PPA operation failed. Rotate on the CPU into
      // the scratch buffer, fully overwriting it, and hand it to draw_bitmap
      // below.
      if (!ensure_rotation_scratch_buffer()) {
        // No way to rotate this area (fallback disabled, or no memory for its
        // buffer): drawing it unrotated would corrupt the frame, so drop it.
        logger_.error_rate_limited(
            "Rotation {} requested but the PPA is unavailable and there is no software "
            "fallback (CONFIG_M5STACK_TAB5_ROTATION_SOFTWARE_FALLBACK / memory); dropping the area",
            static_cast<int>(rotation));
        lv_display_flush_ready(disp);
        return;
      }
      uint32_t w_stride = lv_draw_buf_width_to_stride(ww, cf);
      uint32_t h_stride = lv_draw_buf_width_to_stride(hh, cf);
      {
        // the previous (async) draw may still be reading third_buffer
        std::lock_guard<std::mutex> lock(panel_op_mutex_);
        if (!wait_for_transfer_idle()) {
          lv_display_flush_ready(disp); // wedged: drop this area, keep LVGL alive
          return;
        }
      }
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

  // Pass the draw buffer to the DPI panel driver, asynchronously. Serialized
  // under panel_op_mutex_ and issued only once no transfer is in flight, so a
  // concurrent write_lcd_lines() cannot make this draw fail; then left in
  // flight with lvgl_flush_pending_ set, so the transfer-done ISR signals LVGL
  // (lv_display_flush_ready) the moment the copy finishes while LVGL already
  // renders the next area. The next panel operation, whichever thread issues
  // it, waits for this copy first (wait_for_transfer_idle()).
  //
  // A rejected/failed draw gets no completion: signal LVGL here instead, so it
  // never waits forever.
  {
    std::lock_guard<std::mutex> lock(panel_op_mutex_);
    if (!wait_for_transfer_idle()) {
      // A direct write's transfer is wedged (LVGL's own previous flush cannot
      // be: LVGL waits for it before flushing again). Drop this area rather
      // than issue a draw on top of it, and keep LVGL alive.
      lv_display_flush_ready(disp);
      return;
    }
    // Publish the pending state before the draw: with the CPU copy path (no
    // DMA2D) the driver invokes the completion callback synchronously from
    // inside draw_bitmap, on this thread.
    // Publish the display pointer before the pending flag (release), so the
    // ISR's acquire on the flag always sees the right display.
    lvgl_flush_disp_ = disp;
    lvgl_flush_pending_.store(true, std::memory_order_release);
    transfer_in_flight_.store(true, std::memory_order_release);
    esp_err_t err = esp_lcd_panel_draw_bitmap(lcd_handles_.panel, offsetx1, offsety1, offsetx2 + 1,
                                              offsety2 + 1, px_map);
    if (err != ESP_OK) {
      transfer_in_flight_.store(false, std::memory_order_release);
      lvgl_flush_pending_.store(false, std::memory_order_relaxed);
      logger_.warn_rate_limited("draw_bitmap failed: {}", esp_err_to_name(err));
      lv_display_flush_ready(disp);
    }
  }
}

bool IRAM_ATTR M5StackTab5::notify_lvgl_flush_ready(esp_lcd_panel_handle_t panel,
                                                    esp_lcd_dpi_panel_event_data_t *edata,
                                                    void *user_ctx) {
  espp::M5StackTab5 *tab5 = static_cast<espp::M5StackTab5 *>(user_ctx);
  if (tab5 == nullptr) {
    return false;
  }

  // ISR context (or, on the CPU copy path, called synchronously from inside
  // draw_bitmap on the issuing thread): the draw_bitmap transfer is complete.
  BaseType_t higher_priority_task_woken = pdFALSE;
  tab5->complete_transfer(&higher_priority_task_woken);
  return higher_priority_task_woken == pdTRUE;
}

bool IRAM_ATTR M5StackTab5::notify_ppa_trans_done(ppa_client_handle_t client,
                                                  ppa_event_data_t *edata, void *user_data) {
  (void)client;
  (void)edata;
  auto *tab5 = static_cast<espp::M5StackTab5 *>(user_data);
  if (tab5 == nullptr) {
    return false;
  }
  // ISR context (the PPA / 2D-DMA transfer-done interrupt): the rotation
  // flush() queued into the framebuffer is complete.
  BaseType_t higher_priority_task_woken = pdFALSE;
  tab5->complete_transfer(&higher_priority_task_woken);
  return higher_priority_task_woken == pdTRUE;
}

// The completion callbacks below call lv_display_flush_ready(), which lives in
// flash: they must never run while the flash cache is disabled. Refuse the
// configurations that would make the DSI / PPA (2D-DMA) ISRs run in that state
// rather than ship a firmware that hard-faults on the first SPI-flash write
// during a flush.
#if defined(CONFIG_LCD_DSI_ISR_CACHE_SAFE) && CONFIG_LCD_DSI_ISR_CACHE_SAFE
#error                                                                                             \
    "m5stack-tab5: CONFIG_LCD_DSI_ISR_CACHE_SAFE runs the DSI transfer-done callback with the flash cache disabled, but it calls lv_display_flush_ready() (in flash); disable it"
#endif
#if defined(CONFIG_DMA2D_ISR_IRAM_SAFE) && CONFIG_DMA2D_ISR_IRAM_SAFE
#error                                                                                             \
    "m5stack-tab5: CONFIG_DMA2D_ISR_IRAM_SAFE runs the PPA transfer-done callback with the flash cache disabled, but it calls lv_display_flush_ready() (in flash); disable it"
#endif

void IRAM_ATTR M5StackTab5::complete_transfer(BaseType_t *higher_priority_task_woken) {
  // The transfer in flight (a DMA2D draw_bitmap copy or a PPA rotation) is
  // complete: release LVGL if this was a flush()'s transfer - only flush()
  // sets lvgl_flush_pending_, so a direct write_lcd_lines() completion is
  // never mistaken for an LVGL flush completion - and give the token a direct
  // write (or the next wait_for_transfer_idle()) is waiting on. The in-flight
  // flag is NOT cleared here: the task that consumes the token clears it
  // (wait_for_transfer_idle() / draw_and_wait()), so flag and token can never
  // disagree across cores.
  // lv_display_flush_ready() only clears two flags on the display, which is
  // what the pre-serialization code called from the DSI ISR as well. It lives
  // in flash, so these callbacks must not run while the flash cache is
  // disabled: the #error guards above refuse the configurations that would
  // (CONFIG_LCD_DSI_ISR_CACHE_SAFE for the DSI callback,
  // CONFIG_DMA2D_ISR_IRAM_SAFE for the PPA's), both off by default.
  // acquire pairs with the release store of the flag in flush(), ordering the
  // display pointer read after it
  if (lvgl_flush_pending_.exchange(false, std::memory_order_acq_rel)) {
    lv_display_flush_ready(lvgl_flush_disp_);
  }
  if (draw_done_sem_ != nullptr) {
    // Usually an ISR (the 2D-DMA / PPA transfer-done interrupts), but the DPI
    // driver's CPU copy path (no DMA2D: the ILI9881 variant, or the DMA2D
    // fallback) invokes on_color_trans_done synchronously from inside
    // draw_bitmap on the issuing task: use the matching semaphore API.
    if (xPortInIsrContext()) {
      xSemaphoreGiveFromISR(draw_done_sem_, higher_priority_task_woken);
    } else {
      xSemaphoreGive(draw_done_sem_);
    }
  }
}

bool IRAM_ATTR M5StackTab5::notify_frame_done(esp_lcd_panel_handle_t panel,
                                              esp_lcd_dpi_panel_event_data_t *edata,
                                              void *user_ctx) {
  auto *tab5 = static_cast<espp::M5StackTab5 *>(user_ctx);
  if (tab5 == nullptr || tab5->frame_done_sem_ == nullptr) {
    return false;
  }
  // ISR context (the DMA transfer-done interrupt, or on chips with a DSI
  // bridge VSYNC interrupt and ESP-IDF < v6.0.3, that one): only give the
  // semaphore. Returning true asks the driver to yield to the woken task.
  BaseType_t higher_priority_task_woken = pdFALSE;
  xSemaphoreGiveFromISR(tab5->frame_done_sem_, &higher_priority_task_woken);
  return higher_priority_task_woken == pdTRUE;
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
