#include "m5stack-tab5.hpp"

#include <algorithm>
#include <cstdlib>

#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_ldo_regulator.h>

using namespace std::chrono_literals;

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

  brightness(100.0f);

  // Perform hardware reset sequence via IO expander
  logger_.info("Performing LCD hardware reset sequence");
  lcd_reset(true); // Assert reset
  std::this_thread::sleep_for(10ms);
  lcd_reset(false); // Release reset
  std::this_thread::sleep_for(120ms);

  // Code from the m5stack_tab5 userdemo:
  esp_err_t ret = ESP_OK;

  /* create MIPI DSI bus first, it will initialize the DSI PHY as well */
  esp_lcd_dsi_bus_config_t bus_config = {
      .bus_id = 0,
      .num_data_lanes = 2,
      .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
      .lane_bit_rate_mbps = 730,
  };
  ret = esp_lcd_new_dsi_bus(&bus_config, &lcd_handles_.mipi_dsi_bus);
  if (ret != ESP_OK) {
    logger_.error("New DSI bus init failed: {}", esp_err_to_name(ret));
  }

  logger_.info("Install MIPI DSI LCD control panel");
  // we use DBI interface to send LCD commands and parameters
  esp_lcd_dbi_io_config_t dbi_config = {
      .virtual_channel = 0,
      .lcd_cmd_bits = 8,   // according to the LCD spec
      .lcd_param_bits = 8, // according to the LCD spec
  };
  ret = esp_lcd_new_panel_io_dbi(lcd_handles_.mipi_dsi_bus, &dbi_config, &lcd_handles_.io);
  if (ret != ESP_OK) {
    logger_.error("New panel IO failed: {}", esp_err_to_name(ret));
    // TODO: free previously allocated resources
    return false;
  }

  // Create DPI panel with M5Stack Tab5 official ILI9881 timing parameters
  if (lcd_handles_.panel == nullptr) {
    logger_.info("Creating MIPI DSI DPI panel with M5Stack Tab5 ILI9881 configuration");
    esp_lcd_dpi_panel_config_t dpi_cfg{};
    memset(&dpi_cfg, 0, sizeof(dpi_cfg));
    dpi_cfg.virtual_channel = 0;
    dpi_cfg.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    dpi_cfg.dpi_clock_freq_mhz = 60;
    dpi_cfg.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
    dpi_cfg.num_fbs = 1;
    // Video timing from M5Stack official example for ILI9881 (the default)
    dpi_cfg.video_timing.h_size = 720;           // 1280;
    dpi_cfg.video_timing.v_size = 1280;          // 720;
    dpi_cfg.video_timing.hsync_back_porch = 140; // From M5Stack ILI9881 config
    dpi_cfg.video_timing.hsync_pulse_width = 40; // From M5Stack ILI9881 config
    dpi_cfg.video_timing.hsync_front_porch = 40; // From M5Stack ILI9881 config
    dpi_cfg.video_timing.vsync_back_porch = 20;  // From M5Stack ILI9881 config
    dpi_cfg.video_timing.vsync_pulse_width = 4;  // From M5Stack ILI9881 config
    dpi_cfg.video_timing.vsync_front_porch = 20; // From M5Stack ILI9881 config
    dpi_cfg.flags.use_dma2d = true;

    logger_.info("Creating DPI panel with resolution {}x{}", dpi_cfg.video_timing.h_size,
                 dpi_cfg.video_timing.v_size);
    esp_err_t err = esp_lcd_new_panel_dpi(lcd_handles_.mipi_dsi_bus, &dpi_cfg, &lcd_handles_.panel);
    if (err != ESP_OK) {
      logger_.error("Failed to create MIPI DSI DPI panel: {}", esp_err_to_name(err));
      return false;
    }
  }

  // logger_.info("Install LCD driver of ili9881c");
  // esp_lcd_dpi_panel_config_t dpi_config{};
  // memset(&dpi_config, 0, sizeof(dpi_config));
  // dpi_config.virtual_channel = 0;
  // dpi_config.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
  // dpi_config.dpi_clock_freq_mhz = 60;
  // dpi_config.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
  // dpi_config.num_fbs = 1;
  // dpi_config.video_timing.h_size = display_width_;
  // dpi_config.video_timing.v_size = display_height_;
  // dpi_config.video_timing.hsync_back_porch = 140;
  // dpi_config.video_timing.hsync_pulse_width = 40;
  // dpi_config.video_timing.hsync_front_porch = 40;
  // dpi_config.video_timing.vsync_back_porch = 20;
  // dpi_config.video_timing.vsync_pulse_width = 4;
  // dpi_config.video_timing.vsync_front_porch = 20;
  // dpi_config.flags.use_dma2d = true;

  // ili9881c_vendor_config_t vendor_config = {
  //     .init_cmds = tab5_lcd_ili9881c_specific_init_code_default,
  //     .init_cmds_size = sizeof(tab5_lcd_ili9881c_specific_init_code_default) /
  //                       sizeof(tab5_lcd_ili9881c_specific_init_code_default[0]),
  //     .mipi_config =
  //         {
  //             .dsi_bus = lcd_handles_.mipi_dsi_bus,
  //             .dpi_config = &dpi_config,
  //             .lane_num = 2,
  //         },
  // };

  // const esp_lcd_panel_dev_config_t lcd_dev_config = {
  //     .reset_gpio_num = -1,
  //     .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
  //     .data_endian = LCD_RGB_DATA_ENDIAN_BIG,
  //     .bits_per_pixel = 16,
  //     .flags =
  //         {
  //             .reset_active_high = 1,
  //         },
  //     .vendor_config = &vendor_config,
  // };

  // ESP_ERROR_CHECK(esp_lcd_new_panel_ili9881c(lcd_handles_.io, &lcd_dev_config,
  // &lcd_handles_.panel)); ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_handles_.panel));
  // ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_handles_.panel));
  //  ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_handles_.panel, false, true));

  // Now initialize DisplayDriver for any additional configuration
  logger_.info("Initializing DisplayDriver with DSI configuration");
  using namespace std::placeholders;
  DisplayDriver::initialize(espp::display_drivers::Config{
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
  });

  // ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_handles_.panel, true));

  // call init on the panel
  lcd_handles_.panel->init(lcd_handles_.panel);

  logger_.info("Display initialized with resolution {}x{}", display_width_, display_height_);

  logger_.info("Register DPI panel event callback for LVGL flush ready notification");
  esp_lcd_dpi_panel_event_callbacks_t cbs = {
      .on_color_trans_done = &M5StackTab5::notify_lvgl_flush_ready,
      .on_refresh_done = nullptr,
  };
  ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(lcd_handles_.panel, &cbs, this));

  logger_.info("M5Stack Tab5 LCD initialization completed successfully");
  return true;
}

bool M5StackTab5::initialize_display(size_t pixel_buffer_size) {
  logger_.info("Initializing LVGL display with pixel buffer size: {} pixels", pixel_buffer_size);
  if (!display_) {
    display_ = std::make_shared<Display<Pixel>>(
        Display<Pixel>::LvglConfig{.width = display_width_,
                                   .height = display_height_,
                                   .flush_callback = std::bind_front(&M5StackTab5::flush, this),
                                   .rotation_callback = DisplayDriver::rotate,
                                   .rotation = rotation},
        Display<Pixel>::OledConfig{
            .set_brightness_callback =
                [this](float brightness) { this->brightness(brightness * 100.0f); },
            .get_brightness_callback = [this]() { return this->brightness() / 100.0f; }},
        Display<Pixel>::DynamicMemoryConfig{
            .pixel_buffer_size = pixel_buffer_size,
            .double_buffered = true,
            .allocation_flags = MALLOC_CAP_8BIT | MALLOC_CAP_DMA,
        },
        Logger::Verbosity::WARN);
  }

  logger_.info("LVGL display initialized");
  return true;
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

void IRAM_ATTR M5StackTab5::flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  // Note: This function may be called from ISR context via DPI callback
  // Avoid using floating-point operations, logging, or other coprocessor functions

  if (lcd_handles_.panel == nullptr) {
    lv_display_flush_ready(disp);
    return;
  }

  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;

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
