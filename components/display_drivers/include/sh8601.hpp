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
    writeCommand = config.lcd_write;
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
    display_drivers::LcdInitCmd gc_init_cmds[] = {
        {0x11, {0}, 0, 120},   // sleep out
        {0x13, {0}, 0, 0},     // normal mode
        {static_cast<uint8_t>(config.invert_colors ? Command::invon : Command::invoff), {0}, 0, 0}, // inversion
        // {0x3A, {0x05}, 1, 0},  // color mode 16 bit
        {0x29, {0}, 0, 0},     // display on
        {0x53, {0x28}, 1, 0},  // write CTRL display
        {0x51, {0xFF}, 1, 10}, // brightness normal mode
        {0, {0}, 0xff},        // end of commands
    };

    // send the init commands
    send_commands(gc_init_cmds);
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
    const uint8_t command[] = {static_cast<uint8_t>(Command::madctl), data};
    writeCommand(command, 1, 0);
  }

  /**
   * @brief Flush the pixel data for the provided area to the display.
   * @param *drv Pointer to the LVGL display driver.
   * @param *area Pointer to the structure describing the pixel area.
   * @param *color_map Pointer to array of colors to flush to the display.
   */
  static void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *color_map) {
    fill(disp, area, color_map, (1 << static_cast<int>(display_drivers::Flags::FLUSH_BIT)));
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
    uint8_t data[5] = {0};

    int offset_x = 0;
    int offset_y = 0;
    get_offset_rotated(offset_x, offset_y);

    uint16_t start_x = xs + offset_x;
    uint16_t end_x = xe + offset_x;
    uint16_t start_y = ys + offset_y;
    uint16_t end_y = ye + offset_y;

    // Set the column (x) start / end addresses
    // send_command((uint8_t)Command::caset);
    data[0] = static_cast<uint8_t>(Command::caset);
    data[1] = (start_x >> 8) & 0xFF;
    data[2] = start_x & 0xFF;
    data[3] = (end_x >> 8) & 0xFF;
    data[4] = end_x & 0xFF;
    // send_data(data, 4);
    std::scoped_lock lock{spi_mutex_};
    writeCommand(data, 4, 0);

    // Set the row (y) start / end addresses
    // send_command((uint8_t)Command::raset);
    data[0] = static_cast<uint8_t>(Command::paset);
    data[1] = (start_y >> 8) & 0xFF;
    data[2] = start_y & 0xFF;
    data[3] = (end_y >> 8) & 0xFF;
    data[4] = end_y & 0xFF;
    // send_data(data, 4);
    writeCommand(data, 4, 0);
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
    // lv_draw_sw_rgb565_swap(color_map, lv_area_get_width(area) * lv_area_get_height(area));
    if (lcd_send_lines_) {
      int offset_x = 0;
      int offset_y = 0;
      get_offset_rotated(offset_x, offset_y);

      std::scoped_lock lock{spi_mutex_};
      lcd_send_lines_(area->x1 + offset_x, area->y1 + offset_y, area->x2 + offset_x,
                      area->y2 + offset_y, color_map, flags);
    } else {
      // set_drawing_area(area);
      // send_command(Command::ramwr);
      // uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);
      // send_data(color_map, size * 2, flags);
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
    // send_command(Command::ramwr);
    uint32_t size = width * height;
    static constexpr int max_bytes_to_send = 1024 * 2;
    uint16_t color_data[max_bytes_to_send];
    memset(color_data, color, max_bytes_to_send * sizeof(uint16_t));
    for (int i = 0; i < size; i += max_bytes_to_send) {
      int num_bytes = std::min((int)(size - i), (int)(max_bytes_to_send));
      // send_data((uint8_t *)color_data, num_bytes * 2);
    }
  }

  static void send_commands(display_drivers::LcdInitCmd *commands) {
    using namespace std::chrono_literals;
    // Send all the commands
    uint16_t index = 0;
    uint8_t data[10] = {0};
    while (commands[index].length != 0xff) {
      data[0] = commands[index].command;
      const auto len = commands[index].length;
      if (len > 0) {
        memcpy(&data[1], commands[index].data, len);
      }

      std::scoped_lock lock{spi_mutex_};
      writeCommand(data, len, 0);
      std::this_thread::sleep_for(commands[index].delay_ms * 1ms);
      index++;
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

protected:
  static inline display_drivers::write_fn writeCommand;
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
};
} // namespace espp
