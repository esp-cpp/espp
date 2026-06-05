#include "byte90.hpp"

#include <array>

using namespace espp;

////////////////////////
// Display Functions //
////////////////////////

// the user flag for the callbacks does two things:
// 1. Provides the GPIO level for the data/command pin, and
// 2. Sets some bits for other signaling (such as LVGL FLUSH)
static constexpr int FLUSH_BIT = (1 << (int)espp::display_drivers::Flags::FLUSH_BIT);
static constexpr int DC_LEVEL_BIT = (1 << (int)espp::display_drivers::Flags::DC_LEVEL_BIT);

static void IRAM_ATTR lcd_spi_flush_ready(uint32_t) {
  lv_display_t *disp = lv_display_get_default();
  lv_display_flush_ready(disp);
}

bool Byte90::initialize_lcd() {
  if (lcd_ || lcd_spi_) {
    logger_.warn("LCD already initialized, not initializing again!");
    return false;
  }

  logger_.info("Initializing LCD");

  lcd_spi_ = std::make_unique<Spi>(Spi::Config{
      .host = spi_num,
      .sclk_io_num = spi_sclk_io,
      .mosi_io_num = spi_mosi_io,
      .miso_io_num = GPIO_NUM_NC,
      .max_transfer_sz = SPI_MAX_TRANSFER_BYTES,
      .log_level = get_log_level(),
  });
  lcd_ = std::make_unique<SpiPanelIo>(SpiPanelIo::Config{
      .spi = lcd_spi_.get(),
      .device_config =
          {
              .mode = 0,
              .clock_speed_hz = lcd_clock_speed,
              .input_delay_ns = 0,
              .cs_io_num = lcd_cs_io,
              .queue_size = spi_queue_size,
              .flags = SPI_DEVICE_NO_DUMMY | SPI_DEVICE_3WIRE,
          },
      .data_command_io = lcd_dc_io,
      .data_command_bit_mask = DC_LEVEL_BIT,
      .post_transaction_callback_bit_mask = FLUSH_BIT,
      .post_transaction_callback = lcd_spi_flush_ready,
      .log_level = get_log_level(),
  });
  if (!lcd_->initialized()) {
    lcd_.reset();
    lcd_spi_.reset();
    return false;
  }

  display_driver_ = std::make_shared<DisplayDriver>(
      espp::display_drivers::Config{.panel_io = lcd_.get(),
                                    .write_command = nullptr,
                                    .read_command = nullptr,
                                    .lcd_send_lines = nullptr,
                                    .reset_pin = lcd_reset_io,
                                    .data_command_pin = lcd_dc_io,
                                    .reset_value = reset_value,
                                    .invert_colors = invert_colors,
                                    .swap_color_order = swap_color_order,
                                    .swap_xy = swap_xy,
                                    .mirror_x = mirror_x,
                                    .mirror_y = mirror_y,
                                    .mirror_portrait = mirror_portrait});
  if (!display_driver_ || !display_driver_->initialize()) {
    display_driver_.reset();
    lcd_.reset();
    lcd_spi_.reset();
    return false;
  }
  return true;
}

bool Byte90::initialize_display(size_t pixel_buffer_size) {
  if (!lcd_) {
    logger_.error(
        "LCD not initialized, you must call initialize_lcd() before initialize_display()!");
    return false;
  }
  if (display_) {
    logger_.warn("Display already initialized, not initializing again!");
    return false;
  }

  logger_.info("Initializing display with pixel buffer size: {}", pixel_buffer_size);

  // initialize the display / lvgl
  using namespace std::chrono_literals;
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
              [this](float brightness) {
                if (display_driver_) {
                  display_driver_->set_brightness(brightness);
                }
              },
          .get_brightness_callback =
              [this]() { return display_driver_ ? display_driver_->get_brightness() : 0.0f; }},
      Display<Pixel>::DynamicMemoryConfig{
          .pixel_buffer_size = pixel_buffer_size,
          .double_buffered = true,
          .allocation_flags = MALLOC_CAP_8BIT | MALLOC_CAP_DMA,
      });

  frame_buffer0_ =
      (uint8_t *)heap_caps_malloc(frame_buffer_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  frame_buffer1_ =
      (uint8_t *)heap_caps_malloc(frame_buffer_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  return true;
}

std::shared_ptr<espp::Display<Byte90::Pixel>> Byte90::display() const { return display_; }

void Byte90::write_lcd_frame(const uint16_t xs, const uint16_t ys, const uint16_t width,
                             const uint16_t height, uint8_t *data) {
  if (!display_driver_) {
    return;
  }
  if (data) {
    // have data, fill the area with the color data
    lv_area_t area{.x1 = (lv_coord_t)(xs),
                   .y1 = (lv_coord_t)(ys),
                   .x2 = (lv_coord_t)(xs + width - 1),
                   .y2 = (lv_coord_t)(ys + height - 1)};
    display_driver_->fill(nullptr, &area, data);
  } else {
    // don't have data, so clear the area (set to 0)
    display_driver_->clear(xs, ys, width, height);
  }
}

void Byte90::write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data,
                             uint32_t user_data) {
  if (!lcd_) {
    return;
  }
  size_t length = (xe - xs + 1) * (ye - ys + 1) * lcd_bytes_per_pixel;
  if (length == 0) {
    logger_.error("lcd_send_lines: Bad length: ({},{}) to ({},{})", xs, ys, xe, ye);
    return;
  }
  lcd_->wait();
  std::array<uint8_t, 2> window = {
      static_cast<uint8_t>(xs & 0xff),
      static_cast<uint8_t>(xe & 0xff),
  };
  lcd_->queue_command(static_cast<uint8_t>(DisplayDriver::Command::caset));
  lcd_->queue_data(window);
  window = {
      static_cast<uint8_t>(ys & 0xff),
      static_cast<uint8_t>(ye & 0xff),
  };
  lcd_->queue_command(static_cast<uint8_t>(DisplayDriver::Command::raset));
  lcd_->queue_data(window);
  lcd_->queue_command(static_cast<uint8_t>(DisplayDriver::Command::ramwr));
  lcd_->queue_pixels(data, length, user_data);
}

Byte90::Pixel *Byte90::vram0() const {
  if (!display_) {
    return nullptr;
  }
  return display_->vram0();
}

Byte90::Pixel *Byte90::vram1() const {
  if (!display_) {
    return nullptr;
  }
  return display_->vram1();
}

uint8_t *Byte90::frame_buffer0() const { return frame_buffer0_; }

uint8_t *Byte90::frame_buffer1() const { return frame_buffer1_; }

void Byte90::brightness(float brightness) {
  brightness = std::clamp(brightness, 0.0f, 100.0f) / 100.0f;
  // display expects a value between 0 and 1
  display_->set_brightness(brightness);
}

float Byte90::brightness() const {
  // display returns a value between 0 and 1
  return display_->get_brightness() * 100.0f;
}

size_t Byte90::rotated_display_width() const {
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

size_t Byte90::rotated_display_height() const {
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
