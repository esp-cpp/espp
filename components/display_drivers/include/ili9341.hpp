#pragma once

#include <mutex>

#include "display_drivers.hpp"

namespace espp {
/**
 * @brief Display driver for the ILI9341 display controller.
 *
 *   This code is modified from
 *   https://github.com/lvgl/lvgl_esp32_drivers/blob/master/lvgl_tft/ili9341.c
 *   and
 *   https://github.com/espressif/esp-dev-kits/blob/master/esp32-s2-hmi-devkit-1/components/screen/controller_driver/ili9341/ili9341.c
 *
 *   See also:
 *   https://github.com/espressif/esp-bsp/blob/master/components/lcd/esp_lcd_ili9341/esp_lcd_ili9341.c
 *
 * \section ili9341_wrover_cfg WROVER-KIT ILI9341 Config
 * \snippet display_drivers_example.cpp wrover_kit_config example
 * \section ili9341_ex1 ili9341 Example
 * \snippet display_drivers_example.cpp display_drivers example
 */
class Ili9341 {
public:
  enum class Command : uint8_t {
    invoff = 0x20,  // display inversion off
    invon = 0x21,   // display inversion on
    gamset = 0x26,  // gamma set
    dispoff = 0x28, // display off
    dispon = 0x29,  // display on
    caset = 0x2a,   // column address set
    raset = 0x2b,   // row address set
    ramwr = 0x2c,   // ram write
    rgbset = 0x2d,  // color setting for 4096, 64k and 262k colors
    ramrd = 0x2e,   // ram read
    madctl = 0x36,  // memory data access control
    idmoff = 0x38,  // idle mode off
    idmon = 0x39,   // idle mode on
    ramwrc = 0x3c,  // memory write continue (st7789v)
    ramrdc = 0x3e,  // memory read continue (st7789v)
    colmod = 0x3a,  // color mode - pixel format
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
    mirror_x_ = config.mirror_x;
    mirror_y_ = config.mirror_y;
    mirror_portrait_ = config.mirror_portrait;
    swap_xy_ = config.swap_xy;
    swap_color_order_ = config.swap_color_order;

    // Initialize display pins
    display_drivers::init_pins(reset_pin_, dc_pin_, config.reset_value);

    uint8_t madctl = 0x00;
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
    display_drivers::LcdInitCmd ili_init_cmds[] = {
        {0xCF, {0x00, 0x83, 0X30}, 3},
        {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
        {0xE8, {0x85, 0x01, 0x79}, 3},
        {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
        {0xF7, {0x20}, 1},
        {0xEA, {0x00, 0x00}, 2},
        {0xC0, {0x26}, 1},
        {0xC1, {0x11}, 1},
        {0xC5, {0x35, 0x3E}, 2},
        {0xC7, {0xBE}, 1},
        {0x36, {madctl}, 1},
        {0x3A, {0x55}, 1},
        {0xB1, {0x00, 0x1B}, 2},
        {0xF2, {0x08}, 1},
        {0x26, {0x01}, 1},
        {0xE0,
         {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00},
         15},
        {0XE1,
         {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F},
         15},
        {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
        {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
        {0x2C, {0}, 0},
        {0xB7, {0x07}, 1},
        {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
        {0x11, {0}, 0x80},
        {0x29, {0}, 0x80},
        {0, {0}, 0xff},
    };

    // send the init commands
    send_commands(ili_init_cmds);

    // configure the display color configuration
    if (config.invert_colors) {
      send_command(0x21);
    } else {
      send_command(0x20);
    }
  }

  /**
   * @brief Set the display rotation.
   * @param rotation New display rotation.
   */
  static void rotate(const DisplayRotation &rotation) {
    uint8_t data = 0x00;
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

    int offset_x = 0;
    int offset_y = 0;
    get_offset_rotated(offset_x, offset_y);

    uint16_t start_x = xs + offset_x;
    uint16_t end_x = xe + offset_x;
    uint16_t start_y = ys + offset_y;
    uint16_t end_y = ye + offset_y;

    // Set the column (x) start / end addresses
    send_command(Command::caset);
    data[0] = (start_x >> 8) & 0xFF;
    data[1] = start_x & 0xFF;
    data[2] = (end_x >> 8) & 0xFF;
    data[3] = end_x & 0xFF;
    send_data(data, 4);

    // Set the row (y) start / end addresses
    send_command(Command::raset);
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
      int offset_x = 0;
      int offset_y = 0;
      get_offset_rotated(offset_x, offset_y);
      lcd_send_lines_(area->x1 + offset_x, area->y1 + offset_y, area->x2 + offset_x,
                      area->y2 + offset_y, color_map, flags);
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
  static display_drivers::write_fn lcd_write_;
  static display_drivers::send_lines_fn lcd_send_lines_;
  static gpio_num_t reset_pin_;
  static gpio_num_t dc_pin_;
  static int offset_x_;
  static int offset_y_;
  static bool mirror_x_;
  static bool mirror_y_;
  static bool mirror_portrait_;
  static bool swap_xy_;
  static bool swap_color_order_;
  static std::mutex spi_mutex_;
};
} // namespace espp
