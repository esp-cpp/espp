#pragma once

#include <mutex>

#include "display_drivers.hpp"

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
class Gc9a01 {
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
    raset = 0x2b, // row address set
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

    // init the display
    auto init_commands = std::to_array<display_drivers::DisplayInitCmd<>>({
        {0xEF},
        {0xEB, {0x14}},
        {0xFE},
        {0xEF},
        {0xEB, {0x14}},
        {0x84, {0x40}},
        {0x85, {0xFF}},
        {0x86, {0xFF}},
        {0x87, {0xFF}},
        {0x88, {0x0A}},
        {0x89, {0x21}},
        {0x8A, {0x00}},
        {0x8B, {0x80}},
        {0x8C, {0x01}},
        {0x8D, {0x01}},
        {0x8E, {0xFF}},
        {0x8F, {0xFF}},
        {0xB6, {0x00, 0x20}},
        // call orientation
        {0x36, {madctl}},
        {0x3A, {0x05}},
        {0x90, {0x08, 0x08, 0X08, 0X08}},
        {0xBD, {0x06}},
        {0xBC, {0x00}},
        {0xFF, {0x60, 0x01, 0x04}},
        {0xC3, {0x13}},
        {0xC4, {0x13}},
        {0xC9, {0x22}},
        {0xBE, {0x11}},
        {0xE1, {0x10, 0x0E}},
        {0xDF, {0x21, 0x0C, 0x02}},
        {0xF0, {0x45, 0x09, 0x08, 0x08, 0x26, 0x2A}},
        {0xF1, {0x43, 0x70, 0x72, 0x36, 0x37, 0x6F}},
        {0xF2, {0x45, 0x09, 0x08, 0x08, 0x26, 0x2A}},
        {0xF3, {0x43, 0x70, 0x72, 0x36, 0x37, 0x6F}},
        {0xED, {0x1B, 0x0B}},
        {0xAE, {0x77}},
        {0xCD, {0x63}},
        {0x70, {0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0X08, 0x03}},
        {0xE8, {0x34}},
        {0x62, {0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0X0F, 0x71, 0xEF, 0x70, 0x70}},
        {0x63, {0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0X13, 0x71, 0xF3, 0x70, 0x70}},
        {0x64, {0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07}},
        {0x66, {0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0X00, 0x00, 0x00}},
        {0x67, {0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0X10, 0x32, 0x98}},
        {0x74, {0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00}},
        {0x98, {0x3E, 0x07}},
        {0x35},
        {0x21},
        {0x11, {}, 100},
        {0x29, {}, 100},
    });

    // send the init commands
    send_commands(init_commands);

    // configure the display color configuration
    if (config.invert_colors) {
      write_command_(static_cast<uint8_t>(Command::invon), {}, 0);
    } else {
      write_command_(static_cast<uint8_t>(Command::invoff), {}, 0);
    }
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
    write_command_(static_cast<uint8_t>(Command::madctl), {&data, 1}, 0);
  }

  /**
   * @brief Flush the pixel data for the provided area to the display.
   * @param *disp Pointer to the LVGL display.
   * @param *area Pointer to the structure describing the pixel area.
   * @param *color_map Pointer to array of colors to flush to the display.
   */
  static void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *color_map) {
    fill(disp, area, color_map, (1 << (int)display_drivers::Flags::FLUSH_BIT));
  }

  /**
   * @brief Set the drawing area for the display, resets the cursor to the
   *        starting position of the area.
   * @param *area Pointer to lv_area_t strcuture with start/end x/y
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
    std::array<uint8_t, 4> data;

    int offset_x = 0;
    int offset_y = 0;
    get_offset_rotated(offset_x, offset_y);

    uint16_t start_x = xs + offset_x;
    uint16_t end_x = xe + offset_x;
    uint16_t start_y = ys + offset_y;
    uint16_t end_y = ye + offset_y;

    // Set the column (x) start / end addresses
    data[0] = (start_x >> 8) & 0xFF;
    data[1] = start_x & 0xFF;
    data[2] = (end_x >> 8) & 0xFF;
    data[3] = end_x & 0xFF;
    write_command_(static_cast<uint8_t>(Command::caset), data, 0);

    // Set the row (y) start / end addresses
    data[0] = (start_y >> 8) & 0xFF;
    data[1] = start_y & 0xFF;
    data[2] = (end_y >> 8) & 0xFF;
    data[3] = end_y & 0xFF;
    write_command_(static_cast<uint8_t>(Command::raset), data, 0);
  }

  /**
   * @brief Fill the display area with the provided color map.
   * @param *disp Pointer to the LVGL display.
   * @param *area Pointer to the structure describing the pixel area.
   * @param *color_map Pointer to array of colors to fill the display with.
   * @param flags uint32_t user data / flags to pass to the lcd_write transfer function.
   */
  static void fill(lv_display_t *disp, const lv_area_t *area, uint8_t *color_map,
                   uint32_t flags = 0) {
    std::scoped_lock lock{spi_mutex_};
    lv_draw_sw_rgb565_swap(color_map, lv_area_get_width(area) * lv_area_get_height(area));
    if (lcd_send_lines_) {
      int offset_x = 0;
      int offset_y = 0;
      get_offset_rotated(offset_x, offset_y);
      lcd_send_lines_(area->x1 + offset_x, area->y1 + offset_y, area->x2 + offset_x,
                      area->y2 + offset_y, color_map, flags);
    } else {
      set_drawing_area(area);
      uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);
      write_command_(static_cast<uint8_t>(Command::ramwr), {color_map, size * 2}, flags);
    }
  }

  /**
   * @brief Clear the display area, filling it with the provided color.
   * @param x X coordinate of the upper left corner of the display area.
   * @param y Y coordinate of the upper left corner of the display area.
   * @param width Width of the display area to clear.
   * @param height Height of the display area to clear.
   * @param color 16 bit color (default 0x0000) to fill with.
   */
  static void clear(size_t x, size_t y, size_t width, size_t height, uint16_t color = 0x0000) {
    set_drawing_area(x, y, x + width, y + height);

    // Write the color data to controller RAM
    uint32_t size = width * height;
    static constexpr int max_bytes_to_send = 1024 * 2;
    uint16_t color_data[max_bytes_to_send];
    memset(color_data, color, max_bytes_to_send * sizeof(uint16_t));
    for (int i = 0; i < size; i += max_bytes_to_send) {
      size_t num_bytes = std::min(static_cast<int>(size - i), (int)(max_bytes_to_send));
      write_command_(static_cast<uint8_t>(Command::ramwr),
                     {reinterpret_cast<uint8_t *>(color_data), num_bytes * 2}, 0);
    }
  }

  /**
   * @brief Send the provided commands to the display controller.
   * @param commands Array of display_drivers::LcdInitCmd structures.
   */
  static void send_commands(std::span<const display_drivers::DisplayInitCmd<>> commands) {
    using namespace std::chrono_literals;

    for (const auto &[command, parameters, delay_ms] : commands) {
      std::scoped_lock lock{spi_mutex_};
      write_command_(command, parameters, 0);
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
    auto rotation = lv_display_get_rotation(lv_display_get_default());
    switch (rotation) {
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

protected:
  static inline display_drivers::write_command_fn write_command_;
  static inline display_drivers::send_lines_fn lcd_send_lines_;
  static inline gpio_num_t reset_pin_;
  static inline gpio_num_t dc_pin_;
  static inline int offset_x_;
  static inline int offset_y_;
  static inline bool mirror_x_;
  static inline bool mirror_y_;
  static inline bool mirror_portrait_;
  static inline bool swap_xy_;
  static inline bool swap_color_order_;
  static inline std::mutex spi_mutex_;
};
} // namespace espp
