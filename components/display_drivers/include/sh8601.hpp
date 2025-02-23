#pragma once

#include <mutex>

#include "display_drivers.hpp"

#include <display/lv_display_private.h>
#include <draw/sw/blend/lv_draw_sw_blend_to_rgb565.h>

namespace espp {
/**
 * @brief Display driver for the GC9A01 display controller.
 *
 *   This code is modified from
 *   https://github.com/lvgl/lvgl_esp32_drivers/blob/master/lvgl_tft/GC9A01.c
 *
 *   See also:
 *   https://github.com/espressif/esp-bsp/blob/master/components/lcd/esp_lcd_gc9a01/esp_lcd_gc9a01.c
 *
 * \section smartknob_ha_cfg SmartKnob Config
 * \snippet display_drivers_example.cpp smartknob_config example
 * \section gc9a01_ex1 Gc9a01 Example
 * \snippet display_drivers_example.cpp display_drivers example
 */
class Sh8601 {
public:
  enum class Command : uint8_t {
    nop = 0x00,     // no operation
    swreset = 0x01, // software reset
    rddid = 0x04,   // read display id
    rddst = 0x09,   // read display status

    slpin = 0x10,  // sleep in
    slpout = 0x11, // sleep out
    ptlon = 0x12,  // partial mode on
    noron = 0x13,  // normal display mode on

    invoff = 0x20, // display inversion off
    invon = 0x21,  // display inversion on

    dispoff = 0x28, // display off
    dispon = 0x29,  // display on

    caset = 0x2a, // column address set
    paset = 0x2b, // page address set
    ramwr = 0x2c, // ram write

    ptlar = 0x30,   // partial area
    vscrdef = 0x33, // vertical scrolling definition
    teoff = 0x34,   // tearing effect line off
    teon = 0x35,    // tearing effect line on
    madctl = 0x36,  // memory access control
    idmoff = 0x38,  // idle mode off
    idmon = 0x39,   // idle mode on
    colmod = 0x3a,  // color mode - pixel format
    ramwrc = 0x3c,  // memory write continue

    settes = 0x44, // set tear scanline
    gettes = 0x45, // get tear scanline

    wrdpbr = 0x51, // write display brightness

    wrctrldp = 0x53, // write CTRL display

    readid1 = 0xDA, // read ID 1
    readid2 = 0xDB, // read ID 2
    readid3 = 0xDC, // read ID 3

    rgbctrl = 0xb0,  // rgb control
    porctrl = 0xb5,  // porch control
    dpfuctrl = 0xb6, // display function control

    tectrl = 0xba, // ram control

    intrctrl = 0xf6, // interface control

    frctrl = 0xe8,    // frame rate control
    spi2dctrl = 0xe9, // spi 2data control

    pwrctrl1 = 0xc1, // power control 1
    pwrctrl2 = 0xc3, // power control 2
    pwrctrl3 = 0xc4, // power control 3
    pwrctrl4 = 0xc9, // power control 4
    pwrctrl7 = 0xa7, // power control 7

    intren1 = 0xfe, // inter register enable 1
    intren2 = 0xef, // inter register enable 2

    stgamma1 = 0xf0, // set gamma 1
    stgamma2 = 0xf1, // set gamma 2
    stgamma3 = 0xf2, // set gamma 3
    stgamma4 = 0xf3, // set gamma 4
  };
  /**
   * @brief Store the config data and send the initialization commands to the
   *        display controller.
   * @param config display_drivers::Config structure
   */
  static void initialize(const display_drivers::Config &config) {
    // update the static members
    write_command_ = config.write_command;
    lcd_send_lines_ = config.lcd_send_lines;
    reset_pin_ = config.reset_pin;
    dc_pin_ = config.data_command_pin;
    offset_x_ = config.offset_x;
    offset_y_ = config.offset_y;
    mirror_x_ = config.mirror_x;
    mirror_y_ = config.mirror_y;
    mirror_portrait_ = config.mirror_portrait;
    swap_xy_ = config.swap_xy;
    swap_color_order_ = config.swap_color_order;

    // Initialize display pins
    display_drivers::init_pins(reset_pin_, dc_pin_, config.reset_value);

    uint8_t madctl = 0;
    if (swap_color_order_) {
      madctl |= LCD_CMD_BGR_BIT;
    }
    if (mirror_x_) {
      madctl |= LCD_CMD_MX_BIT;
    }
    if (mirror_y_) {
      madctl |= LCD_CMD_MY_BIT;
    }
    if (swap_xy_) {
      madctl |= LCD_CMD_MV_BIT;
    }

    auto init_cmds = std::to_array<display_drivers::LcdInitCmd<Command>>({
        {Command::slpout, {0}, 0, 120},                            // sleep out
        {Command::noron},                                          // normal mode
        {config.invert_colors ? Command::invon : Command::invoff}, // inversion
        {Command::colmod, {0x07}, 1},                              // color mode 24 bit
        {Command::dispon},                                         // display on
        {Command::wrctrldp, {0x28}, 1},                            // write CTRL display
        {Command::wrdpbr, {0xFF}, 1, 10}, // brightness normal mode   // end of commands
    });

    // send the init commands
    send_commands(init_cmds);
  }

  /**
   * @brief Set the display rotation.
   * @param rotation New display rotation.
   */
  static void rotate(const DisplayRotation &rotation) {
    uint8_t data = 0;
    if (swap_color_order_) {
      data |= LCD_CMD_BGR_BIT;
    }
    if (mirror_x_) {
      data |= LCD_CMD_MX_BIT;
    }
    if (mirror_y_) {
      data |= LCD_CMD_MY_BIT;
    }
    if (swap_xy_) {
      data |= LCD_CMD_MV_BIT;
    }

    switch (rotation) {
    case DisplayRotation::LANDSCAPE:
      break;
    case DisplayRotation::PORTRAIT:
      // flip the mx and mv bits (xor)
      if (mirror_portrait_) {
        data ^= (LCD_CMD_MX_BIT | LCD_CMD_MV_BIT);
      } else {
        data ^= (LCD_CMD_MY_BIT | LCD_CMD_MV_BIT);
      }
      break;
    case DisplayRotation::LANDSCAPE_INVERTED:
      // flip the my and mx bits (xor)
      data ^= (LCD_CMD_MY_BIT | LCD_CMD_MX_BIT);
      break;
    case DisplayRotation::PORTRAIT_INVERTED:
      // flip the my and mv bits (xor)
      if (mirror_portrait_) {
        data ^= (LCD_CMD_MY_BIT | LCD_CMD_MV_BIT);
      } else {
        data ^= (LCD_CMD_MX_BIT | LCD_CMD_MV_BIT);
      }
      break;
    }
    std::scoped_lock lock{spi_mutex_};
    write_command_(static_cast<uint8_t>(Command::madctl), &data, 1, 0);
  }

  /**
   * @brief Flush the pixel data for the provided area to the display.
   * @param *drv Pointer to the LVGL display driver.
   * @param *area Pointer to the structure describing the pixel area.
   * @param *color_map Pointer to array of colors to flush to the display.
   */
  static void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *color_map) {
    // No need to swap the colors when in RGB888 mode
#if LV_COLOR_DEPTH == 16
    lv_draw_sw_rgb565_swap(color_map, lv_area_get_width(area) * lv_area_get_height(area));
#endif

    int offset_x = 0;
    int offset_y = 0;
    get_offset_rotated(offset_x, offset_y);

    lcd_send_lines_(area->x1 + offset_x, area->y1 + offset_y, area->x2 + offset_x,
                    area->y2 + offset_y, color_map,
                    (1 << static_cast<int>(display_drivers::Flags::FLUSH_BIT)));
  }

  /**
   * @brief Set the drawing area for the display, resets the cursor to the
   *        starting position of the area.
   * @param area Pointer to lv_area_t strcuture with start/end x/y
   *              coordinates.
   */
  static void set_drawing_area(const lv_area_t *area) {
    set_drawing_area(area->x1, area->y1, area->x2, area->y2);
  }

  /**
   * @brief Set the drawing area for the display, resets the cursor to the
   *        starting position of the area.
   * @param xs Starting x coordinate of the area.
   * @param ys Starting y coordinate of the area.
   * @param xe Ending x coordinate of the area.
   * @param ye Ending y coordinate of the area.
   */
  static void set_drawing_area(size_t xs, size_t ys, size_t xe, size_t ye) {
    uint8_t data[4];

    int offset_x = 0;
    int offset_y = 0;
    get_offset_rotated(offset_x, offset_y);

    const uint16_t start_x = xs + offset_x;
    const uint16_t end_x = xe + offset_x;
    const uint16_t start_y = ys + offset_y;
    const uint16_t end_y = ye + offset_y;

    // Set the column (x) start / end addresses
    data[0] = (start_x >> 8) & 0xFF;
    data[1] = start_x & 0xFF;
    data[2] = (end_x >> 8) & 0xFF;
    data[3] = end_x & 0xFF;
    std::scoped_lock lock{spi_mutex_};
    write_command_(static_cast<uint8_t>(Command::caset), data, 4, 0);

    // Set the row (y) start / end addresses
    data[0] = (start_y >> 8) & 0xFF;
    data[1] = start_y & 0xFF;
    data[2] = (end_y >> 8) & 0xFF;
    data[3] = end_y & 0xFF;
    write_command_(static_cast<uint8_t>(Command::paset), data, 4, 0);
  }

  static void send_commands(std::span<display_drivers::LcdInitCmd<Command>> commands) {
    using namespace std::chrono_literals;

    for (auto [command, data, length, delay_ms] : commands) {
      std::scoped_lock lock{spi_mutex_};
      write_command_(static_cast<uint8_t>(command), data, length, 0);
      std::this_thread::sleep_for(delay_ms * 1ms);
    }
  }

  /**
   * @brief Set the offset (upper left starting coordinate) of the display.
   * @note This modifies internal variables that are used when sending
   *       coordinates / filling parts of the display.
   * @param x New starting x coordinate (so writing to x address 0 later will
   *          actually write to this offset).
   * @param y New starting y coordinate (so writing to y address 0 later will
   *          actually write to this offset).
   */
  static void set_offset(int x, int y) {
    offset_x_ = x;
    offset_y_ = y;
  }

  /**
   * @brief Get the offset (upper left starting coordinate) of the display.
   * @note This returns internal variables that are used when sending
   *       coordinates / filling parts of the display.
   * @param x Reference variable that will be filled with the currently
   *          configured starting x coordinate that was provided in the config
   *          or set by set_offset().
   * @param y Reference variable that will be filled with the currently
   *          configured starting y coordinate that was provided in the config
   *          or set by set_offset().
   */
  static void get_offset(int &x, int &y) {
    x = offset_x_;
    y = offset_y_;
  }

  /**
   * @brief Get the offset (upper left starting coordinate) of the display
   *        after rotation.
   * @note This returns internal variables that are used when sending
   *       coordinates / filling parts of the display.
   * @param x Reference variable that will be filled with the currently
   *          configured starting x coordinate that was provided in the config
   *          or set by set_offset(), updated for the current rotation.
   * @param y Reference variable that will be filled with the currently
   *          configured starting y coordinate that was provided in the config
   *          or set by set_offset(), updated for the current rotation.
   */
  static void get_offset_rotated(int &x, int &y) {
    switch (auto rotation = lv_display_get_rotation(lv_display_get_default())) {
    case LV_DISPLAY_ROTATION_90:
      // intentional fallthrough
    case LV_DISPLAY_ROTATION_270:
      x = offset_y_;
      y = offset_x_;
      break;
    case LV_DISPLAY_ROTATION_0:
      // intentional fallthrough
    case LV_DISPLAY_ROTATION_180:
      // intentional fallthrough
    default:
      x = offset_x_;
      y = offset_y_;
      break;
    }
  }

  static void set_brightness(const float brightness) {
    // Update the local brightness value
    brightness_ = brightness;

    // This display has a 10-bit brightness control
    uint16_t data = brightness * 1023;
    printf("Setting brightness to %f (%d)\n", brightness, data);
    write_command_(static_cast<uint8_t>(Command::wrdpbr), reinterpret_cast<uint8_t *>(&data), 2, 0);
  }

  static void set_brightness(const uint8_t brightness) {
    // Convert the 8-bit brightness to a float
    set_brightness(static_cast<float>(brightness) / 255.0f);
  }

  static float get_brightness() { return brightness_; }

protected:
  static inline display_drivers::write_command_fn write_command_;
  static inline display_drivers::send_lines_fn lcd_send_lines_;
  static inline gpio_num_t reset_pin_;
  static inline gpio_num_t dc_pin_;
  static inline int offset_x_;
  static inline int offset_y_;
  static inline bool mirror_x_ = false;
  static inline bool mirror_y_ = false;
  static inline bool mirror_portrait_ = false;
  static inline bool swap_xy_ = false;
  static inline bool swap_color_order_ = false;
  static inline std::mutex spi_mutex_;
  static inline float brightness_ = 0;
};
} // namespace espp
