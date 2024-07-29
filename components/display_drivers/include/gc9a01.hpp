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
    lcd_write_ = config.lcd_write;
    lcd_send_lines_ = config.lcd_send_lines;
    reset_pin_ = config.reset_pin;
    dc_pin_ = config.data_command_pin;
    offset_x_ = config.offset_x;
    offset_y_ = config.offset_y;
    swap_xy_ = config.swap_xy;

    // Initialize display pins
    display_drivers::init_pins(reset_pin_, dc_pin_, config.reset_value);

    // init the display
    display_drivers::LcdInitCmd gc_init_cmds[] = {
        {0xEF, {0}, 0},
        {0xEB, {0x14}, 1},
        {0xFE, {0}, 0},
        {0xEF, {0}, 0},
        {0xEB, {0x14}, 1},
        {0x84, {0x40}, 1},
        {0x85, {0xFF}, 1},
        {0x86, {0xFF}, 1},
        {0x87, {0xFF}, 1},
        {0x88, {0x0A}, 1},
        {0x89, {0x21}, 1},
        {0x8A, {0x00}, 1},
        {0x8B, {0x80}, 1},
        {0x8C, {0x01}, 1},
        {0x8D, {0x01}, 1},
        {0x8E, {0xFF}, 1},
        {0x8F, {0xFF}, 1},
        {0xB6, {0x00, 0x20}, 2},
        // call orientation
        {0x36, {0x00}, 1},
        {0x3A, {0x05}, 1},
        {0x90, {0x08, 0x08, 0X08, 0X08}, 4},
        {0xBD, {0x06}, 1},
        {0xBC, {0x00}, 1},
        {0xFF, {0x60, 0x01, 0x04}, 3},
        {0xC3, {0x13}, 1},
        {0xC4, {0x13}, 1},
        {0xC9, {0x22}, 1},
        {0xBE, {0x11}, 1},
        {0xE1, {0x10, 0x0E}, 2},
        {0xDF, {0x21, 0x0C, 0x02}, 3},
        {0xF0, {0x45, 0x09, 0x08, 0x08, 0x26, 0x2A}, 6},
        {0xF1, {0x43, 0x70, 0x72, 0x36, 0x37, 0x6F}, 6},
        {0xF2, {0x45, 0x09, 0x08, 0x08, 0x26, 0x2A}, 6},
        {0xF3, {0x43, 0x70, 0x72, 0x36, 0x37, 0x6F}, 6},
        {0xED, {0x1B, 0x0B}, 2},
        {0xAE, {0x77}, 1},
        {0xCD, {0x63}, 1},
        {0x70, {0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0X08, 0x03}, 9},
        {0xE8, {0x34}, 1},
        {0x62, {0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0X0F, 0x71, 0xEF, 0x70, 0x70}, 12},
        {0x63, {0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0X13, 0x71, 0xF3, 0x70, 0x70}, 12},
        {0x64, {0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07}, 7},
        {0x66, {0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0X00, 0x00, 0x00}, 10},
        {0x67, {0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0X10, 0x32, 0x98}, 10},
        {0x74, {0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00}, 7},
        {0x98, {0x3E, 0x07}, 2},
        {0x35, {0}, 0},
        {0x21, {0}, 0},
        {0x11, {0}, 0x80},
        {0x29, {0}, 0x80},
        {0, {0}, 0xff},
    };

    // NOTE: these configurations operates on the MADCTL command / register
    if (config.mirror_x) {
      gc_init_cmds[18].data[0] |= LCD_CMD_MX_BIT;
    }
    if (config.mirror_y) {
      gc_init_cmds[18].data[0] |= LCD_CMD_MY_BIT;
    }
    if (swap_xy_) {
      gc_init_cmds[18].data[0] |= LCD_CMD_MV_BIT;
    }

    // send the init commands
    send_commands(gc_init_cmds);

    // configure the display color configuration
    if (config.invert_colors) {
      send_command(Command::invon);
    } else {
      send_command(Command::invoff);
    }
  }

  /**
   * @brief Set the display rotation.
   * @param rotation New display rotation.
   */
  static void rotate(const DisplayRotation &rotation) {
    uint8_t data = 0;
    switch (rotation) {
    case DisplayRotation::LANDSCAPE:
      data = 0x00;
      break;
    case DisplayRotation::PORTRAIT:
      data |= LCD_CMD_MV_BIT | LCD_CMD_MX_BIT;
      break;
    case DisplayRotation::LANDSCAPE_INVERTED:
      data |= LCD_CMD_MX_BIT | LCD_CMD_MY_BIT;
      break;
    case DisplayRotation::PORTRAIT_INVERTED:
      data |= LCD_CMD_MV_BIT | LCD_CMD_MY_BIT;
      break;
    }
    if (swap_xy_) {
      data |= LCD_CMD_MV_BIT;
    }
    std::scoped_lock lock{spi_mutex_};
    send_command(Command::madctl);
    send_data(&data, 1);
  }

  /**
   * @brief Flush the pixel data for the provided area to the display.
   * @param *drv Pointer to the LVGL display driver.
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
    uint8_t data[4] = {0};

    uint16_t start_x = xs + offset_x_;
    uint16_t end_x = xe + offset_x_;
    uint16_t start_y = ys + offset_y_;
    uint16_t end_y = ye + offset_y_;

    // Set the column (x) start / end addresses
    send_command((uint8_t)Command::caset);
    data[0] = (start_x >> 8) & 0xFF;
    data[1] = start_x & 0xFF;
    data[2] = (end_x >> 8) & 0xFF;
    data[3] = end_x & 0xFF;
    send_data(data, 4);

    // Set the row (y) start / end addresses
    send_command((uint8_t)Command::raset);
    data[0] = (start_y >> 8) & 0xFF;
    data[1] = start_y & 0xFF;
    data[2] = (end_y >> 8) & 0xFF;
    data[3] = end_y & 0xFF;
    send_data(data, 4);
  }

  /**
   * @brief Flush the pixel data for the provided area to the display.
   * @param *drv Pointer to the LVGL display driver.
   * @param *area Pointer to the structure describing the pixel area.
   * @param *color_map Pointer to array of colors to flush to the display.
   * @param flags uint32_t user data / flags to pass to the lcd_write transfer function.
   */
  static void fill(lv_display_t *disp, const lv_area_t *area, uint8_t *color_map,
                   uint32_t flags = 0) {
    std::scoped_lock lock{spi_mutex_};
    lv_draw_sw_rgb565_swap(color_map, lv_area_get_width(area) * lv_area_get_height(area));
    if (lcd_send_lines_) {
      lcd_send_lines_(area->x1 + offset_x_, area->y1 + offset_y_, area->x2 + offset_x_,
                      area->y2 + offset_y_, color_map, flags);
    } else {
      set_drawing_area(area);
      send_command(Command::ramwr);
      uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);
      send_data(color_map, size * 2, flags);
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
    send_command(Command::ramwr);
    uint32_t size = width * height;
    static constexpr int max_bytes_to_send = 1024 * 2;
    uint16_t color_data[max_bytes_to_send];
    memset(color_data, color, max_bytes_to_send * sizeof(uint16_t));
    for (int i = 0; i < size; i += max_bytes_to_send) {
      int num_bytes = std::min((int)(size - i), (int)(max_bytes_to_send));
      send_data((uint8_t *)color_data, num_bytes * 2);
    }
  }

  /**
   * @brief Sends the command, sets flags such that the pre-cb should set the
   *        DC pin to command mode.
   * @param command Command code to send
   */
  static void send_command(uint8_t command) {
    uint16_t flags = 0;
    lcd_write_(&command, 1, flags);
  }

  /**
   * @brief Sends the command, sets flags such that the pre-cb should set the
   *        DC pin to command mode.
   * @param command Command code to send
   */
  static void send_command(Command command) {
    uint16_t flags = 0;
    auto command_ = static_cast<uint8_t>(command);
    lcd_write_(&command_, 1, flags);
  }

  /**
   * @brief Send data to the display. Adds (1<<DC_LEVEL_BIT) to the flags so
   *        that the pre-cb receives it in user data and can configure the DC
   *        pin to data mode.
   * @param data Pointer to array of bytes to be sent
   * @param length Number of bytes of data to send.
   * @param flags Optional (default = 0) flags associated with transfer.
   */
  static void send_data(const uint8_t *data, size_t length, uint32_t flags = 0) {
    flags |= (1 << (int)display_drivers::Flags::DC_LEVEL_BIT);
    lcd_write_(data, length, flags);
  }

  static void send_commands(display_drivers::LcdInitCmd *commands) {
    using namespace std::chrono_literals;
    // Send all the commands
    uint16_t cmd = 0;
    while (commands[cmd].length != 0xff) {
      send_command(commands[cmd].command);
      send_data(commands[cmd].data, commands[cmd].length & 0x1F);
      if (commands[cmd].length & 0x80) {
        std::this_thread::sleep_for(100ms);
      }
      cmd++;
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

protected:
  static display_drivers::write_fn lcd_write_;
  static display_drivers::send_lines_fn lcd_send_lines_;
  static gpio_num_t reset_pin_;
  static gpio_num_t dc_pin_;
  static int offset_x_;
  static int offset_y_;
  static bool swap_xy_;
  static std::mutex spi_mutex_;
};
} // namespace espp
