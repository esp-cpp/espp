#include "m5stack-tab5.hpp"

#include <algorithm>

#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_ldo_regulator.h>

namespace espp {

bool M5StackTab5::initialize_lcd() {
  logger_.info("Initializing M5Stack Tab5 LCD (MIPI-DSI, ILI9881C, {}x{})", display_width_,
               display_height_);

  if (!ioexp_0x43_) {
    if (!initialize_io_expanders()) {
      logger_.error("Failed to init IO expanders for LCD reset");
      return false;
    }
  }

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
    esp_err_t err = esp_ldo_acquire_channel(&phy_pwr_cfg, &phy_pwr_chan);
    if (err != ESP_OK) {
      logger_.error("Failed to acquire MIPI DSI PHY power LDO channel: {}", esp_err_to_name(err));
      return false;
    }
  }

  // Ensure panel reset sequence via IO expander
  lcd_reset(true);
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(20ms);
  lcd_reset(false);
  std::this_thread::sleep_for(120ms);

  // Configure backlight PWM like esp-box
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

    // // for now we're just going to set the gpio
    // gpio_set_direction(lcd_backlight_io, GPIO_MODE_OUTPUT);
  }

  brightness(100.0f);

  // Create MIPI-DSI bus and DBI panel-IO
  if (lcd_handles_.mipi_dsi_bus == nullptr) {
    esp_lcd_dsi_bus_config_t bus_cfg{};
    memset(&bus_cfg, 0, sizeof(bus_cfg));
    bus_cfg.bus_id = 0;
    bus_cfg.num_data_lanes = 2; // Tab5 uses 4 lanes
    bus_cfg.phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT;
    bus_cfg.lane_bit_rate_mbps = 1000; // 720*1280 RGB565 60Hz ~= 884 Mbps, use 1 Gbps for safety
    logger_.info("Creating DSI bus with {} data lanes at {} Mbps", bus_cfg.num_data_lanes,
                 bus_cfg.lane_bit_rate_mbps);
    esp_err_t err = esp_lcd_new_dsi_bus(&bus_cfg, &lcd_handles_.mipi_dsi_bus);
    if (err != ESP_OK) {
      logger_.error("Failed to create DSI bus: {}", esp_err_to_name(err));
      return false;
    }
  }

  if (lcd_handles_.io == nullptr) {
    esp_lcd_dbi_io_config_t io_cfg{};
    memset(&io_cfg, 0, sizeof(io_cfg));
    io_cfg.virtual_channel = 0;
    io_cfg.lcd_cmd_bits = 8;
    io_cfg.lcd_param_bits = 8;
    logger_.info("Creating DSI DBI panel IO with {} cmd bits and {} param bits",
                 io_cfg.lcd_cmd_bits, io_cfg.lcd_param_bits);
    esp_err_t err = esp_lcd_new_panel_io_dbi(lcd_handles_.mipi_dsi_bus, &io_cfg, &lcd_handles_.io);
    if (err != ESP_OK) {
      logger_.error("Failed to create DSI DBI panel IO: {}", esp_err_to_name(err));
      return false;
    }
  }

  if (lcd_handles_.panel == nullptr) {
    esp_lcd_dpi_panel_config_t dpi_cfg{};
    memset(&dpi_cfg, 0, sizeof(dpi_cfg));
    dpi_cfg.virtual_channel = 0;
    dpi_cfg.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    dpi_cfg.dpi_clock_freq_mhz = 80;
    // dpi_cfg.in_color_format = LCD_COLOR_FMT_RGB888;
    dpi_cfg.in_color_format = LCD_COLOR_FMT_RGB565;
    dpi_cfg.video_timing.h_size = 800;
    dpi_cfg.video_timing.v_size = 1280;
    dpi_cfg.video_timing.hsync_back_porch = 140;
    dpi_cfg.video_timing.hsync_pulse_width = 40;
    dpi_cfg.video_timing.hsync_front_porch = 40;
    dpi_cfg.video_timing.vsync_back_porch = 16;
    dpi_cfg.video_timing.vsync_pulse_width = 4;
    dpi_cfg.video_timing.vsync_front_porch = 16;

    // TODO:
    dpi_cfg.flags.use_dma2d = true;

    // ili9881c_vendor_config_t vendor_config = {
    //     .mipi_config = {
    //         .dsi_bus = dsi_bus_,
    //         .dpi_config = &dpi_cfg,
    //         .lane_num = 2,
    //     },
    // };
    // esp_lcd_panel_dev_config_t lcd_dev_config = {
    //     .reset_gpio_num = -1,
    //     .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
    //     .bits_per_pixel = 24,
    //     .vendor_config = &vendor_config,
    // };
    // ESP_ERROR_CHECK(esp_lcd_new_panel_ili9881c(mipi_dbi_io, &lcd_dev_config, &mipi_dpi_panel_));

    logger_.info("Creating MIPI DSI DPI panel with resolution {}x{}", dpi_cfg.video_timing.h_size,
                 dpi_cfg.video_timing.v_size);
    esp_err_t err = esp_lcd_new_panel_dpi(lcd_handles_.mipi_dsi_bus, &dpi_cfg, &lcd_handles_.panel);
    if (err != ESP_OK) {
      logger_.error("Failed to create MIPI DSI DPI panel: {}", esp_err_to_name(err));
      return false;
    }
  }

  // ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_handles_.panel));
  ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_handles_.panel));

  logger_.info("Register DPI panel event callback for LVGL flush ready notification");
  esp_lcd_dpi_panel_event_callbacks_t cbs = {
      .on_color_trans_done = &M5StackTab5::notify_lvgl_flush_ready,
      // .on_refresh_done = &M5StackTab5::monitor_refresh_rate,
  };
  ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(lcd_handles_.panel, &cbs, nullptr));

  logger_.info("DSI bus and panel IO created successfully, starting DisplayDriver initialization");
  using namespace std::placeholders;
  DisplayDriver::initialize(espp::display_drivers::Config{
      .write_command = std::bind_front(&M5StackTab5::dsi_write_command, this),
      .lcd_send_lines = std::bind_front(&M5StackTab5::dsi_write_lcd_lines, this),
      .reset_pin = GPIO_NUM_NC,        // reset handled via IO expander
      .data_command_pin = GPIO_NUM_NC, // DSI has no DC pin
      .reset_value = false,
      .invert_colors = invert_colors,
      .swap_color_order = swap_color_order,
      .offset_x = 0,
      .offset_y = 0,
      .swap_xy = swap_xy,
      .mirror_x = mirror_x,
      .mirror_y = mirror_y,
      .mirror_portrait = false,
  });

  logger_.info("LCD driver initialized (callbacks bound)");
  return true;
}

bool M5StackTab5::initialize_display(size_t pixel_buffer_size) {
  logger_.info("Initializing LVGL display with pixel buffer size: {} pixels", pixel_buffer_size);
  if (!display_) {
    display_ = std::make_shared<Display<Pixel>>(
        typename Display<Pixel>::LvglConfig{.width = display_width_,
                                            .height = display_height_,
                                            .flush_callback =
                                                DisplayDriver::flush, // M5StackTab5::flush,
                                            .rotation_callback = DisplayDriver::rotate,
                                            .rotation = rotation},
        typename Display<Pixel>::OledConfig{
            .set_brightness_callback = [this](float b) { this->brightness(b * 100.0f); },
            .get_brightness_callback = [this]() { return this->brightness() / 100.0f; }},
        typename Display<Pixel>::DynamicMemoryConfig{.pixel_buffer_size = pixel_buffer_size,
                                                     .double_buffered = true,
                                                     .allocation_flags =
                                                         MALLOC_CAP_8BIT | MALLOC_CAP_DMA},
        Logger::Verbosity::WARN);
  }

  lv_display_set_user_data(lv_display_get_default(), lcd_handles_.panel);

  // // set color depth
  // lv_display_set_color_format(lv_display_get_default(),
  //                             LV_COLOR_FORMAT_RGB565);

  logger_.info("LVGL display initialized");
  return true;
}

void M5StackTab5::brightness(float brightness) {
  brightness = std::clamp(brightness, 0.0f, 100.0f);
  if (backlight_) {
    backlight_->set_duty(LEDC_CHANNEL_0, brightness);
  } else {
    gpio_set_level(lcd_backlight_io, brightness > 0 ? 1 : 0);
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
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
  if (panel_handle == nullptr)
    return;
  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;
  // pass the draw buffer to the driver
  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
}

bool M5StackTab5::notify_lvgl_flush_ready(esp_lcd_panel_handle_t panel,
                                          esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx) {
  // lv_display_t *disp = (lv_display_t *)user_ctx;
  // lv_display_flush_ready(disp);
  lv_display_flush_ready(lv_display_get_default());
  return false;
}

void M5StackTab5::dsi_write_command(uint8_t cmd, std::span<const uint8_t> params,
                                    uint32_t /*flags*/) {
  if (!lcd_handles_.io)
    return;
  esp_lcd_panel_io_handle_t io = lcd_handles_.io;
  const void *data_ptr = params.data();
  size_t data_size = params.size();
  logger_.debug("DSI tx_param 0x{:02X} with {} bytes", cmd, data_size);
  esp_err_t err = esp_lcd_panel_io_tx_param(io, (int)cmd, data_ptr, data_size);
  if (err != ESP_OK) {
    logger_.error("DSI tx_param 0x{:02X} failed: {}", cmd, esp_err_to_name(err));
  }
}

void M5StackTab5::dsi_write_lcd_lines(int sx, int sy, int ex, int ey, const uint8_t *color_data,
                                      uint32_t /*flags*/) {
  if (!lcd_handles_.io)
    return;
  esp_lcd_panel_io_handle_t io = lcd_handles_.io;

  // Calculate total number of pixels in
  // the area
  const int width = (ex - sx + 1);
  const int height = (ey - sy + 1);
  const size_t num_pixels = static_cast<size_t>(width) * static_cast<size_t>(height);

  logger_.debug("DSI tx_color for area ({},{})->({},{}) size={} px", sx, sy, ex, ey,
                width * height);

  // Ensure drawing area is set, then stream pixel data via RAMWR using panel IO color API
  lv_area_t area{
      .x1 = (lv_coord_t)sx, .y1 = (lv_coord_t)sy, .x2 = (lv_coord_t)ex, .y2 = (lv_coord_t)ey};
  DisplayDriver::set_drawing_area(&area);

  // RAMWR expects RGB565 little-endian words; DisplayDriver::fill already byte-swapped when needed
  esp_err_t err =
      esp_lcd_panel_io_tx_color(io, (int)DisplayDriver::Command::ramwr, color_data, num_pixels);
  if (err != ESP_OK) {
    logger_.error("DSI tx_color failed for area ({},{})->({},{}) size={} px: {}", sx, sy, ex, ey,
                  num_pixels, esp_err_to_name(err));
  }
}

} // namespace espp
