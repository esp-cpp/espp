#include "qemu.hpp"

#include <esp_lcd_qemu_rgb.h>

namespace espp {

bool Qemu::initialize_display() { return initialize_display(DisplayConfig{}); }

bool Qemu::initialize_display(const DisplayConfig &config) {
  if (display_) {
    logger_.warn("Display already initialized");
    return true;
  }
  if (config.width == 0 || config.height == 0) {
    logger_.error("The panel needs a non-zero size");
    return false;
  }
  logger_.info("Initializing the virtual RGB panel ({}x{}, RGB565)", config.width, config.height);
  // The panel is a QEMU device with its own frame buffer (outside the chip's
  // RAM); esp_lcd_panel_draw_bitmap() copies a window into it and the window
  // shows it. On real hardware its registers do not exist: the driver refuses
  // to create it, which is the "are we in QEMU" test.
  esp_lcd_rgb_qemu_config_t panel_config = {
      .width = static_cast<uint32_t>(config.width),
      .height = static_cast<uint32_t>(config.height),
      .bpp = RGB_QEMU_BPP_16,
  };
  esp_err_t err = esp_lcd_new_rgb_qemu(&panel_config, &panel_);
  if (err != ESP_OK) {
    logger_.error("Could not create the QEMU RGB panel: {} (not running in QEMU?)",
                  esp_err_to_name(err));
    panel_ = nullptr;
    return false;
  }
  err = esp_lcd_panel_init(panel_);
  if (err != ESP_OK) {
    logger_.error("Could not initialize the QEMU RGB panel: {}", esp_err_to_name(err));
    esp_lcd_panel_del(panel_);
    panel_ = nullptr;
    return false;
  }
  display_width_ = config.width;
  display_height_ = config.height;

  const size_t pixel_buffer_size =
      config.pixel_buffer_size != 0 ? config.pixel_buffer_size : config.width * config.height / 10;
  logger_.info("Initializing LVGL display with {} draw buffer{} of {} pixels",
               config.double_buffered ? 2 : 1, config.double_buffered ? "s" : "",
               pixel_buffer_size);
  display_ = std::make_shared<Display<Pixel>>(
      Display<Pixel>::LvglConfig{
          .width = config.width,
          .height = config.height,
          .flush_callback = std::bind_front(&Qemu::flush, this),
          // the virtual panel has no scan-direction control, and the flush
          // writes LVGL's buffers as they are: the display stays unrotated
          .rotation = DisplayRotation::LANDSCAPE,
      },
      // no backlight to drive
      Display<Pixel>::LcdConfig{},
      Display<Pixel>::DynamicMemoryConfig{
          .pixel_buffer_size = pixel_buffer_size,
          .double_buffered = config.double_buffered,
          // draw_bitmap() on this panel is a CPU copy: any RAM will do
          .allocation_flags = MALLOC_CAP_8BIT,
      },
      logger_.get_verbosity());
  logger_.info("LVGL display initialized");
  return true;
}

void Qemu::flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  // LVGL task context: copy the rendered area into the panel's frame buffer.
  // The copy is synchronous (a memcpy into the device's memory), so LVGL can
  // be released as soon as it returns.
  if (panel_ != nullptr) {
    std::lock_guard<std::mutex> lock(panel_mutex_);
    esp_lcd_panel_draw_bitmap(panel_, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);
  }
  lv_display_flush_ready(disp);
}

void Qemu::write_lcd_lines(int xs, int ys, int xe, int ye, const Pixel *data) {
  if (panel_ == nullptr || data == nullptr) {
    return;
  }
  if (xs < 0 || ys < 0 || xe < xs || ye < ys) {
    logger_.error("write_lcd_lines: bad region ({},{}) to ({},{})", xs, ys, xe, ye);
    return;
  }
  std::lock_guard<std::mutex> lock(panel_mutex_);
  esp_lcd_panel_draw_bitmap(panel_, xs, ys, xe + 1, ye + 1, data);
}

} // namespace espp
