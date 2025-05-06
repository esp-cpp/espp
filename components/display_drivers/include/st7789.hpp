#pragma once

#include <memory>
#include <mutex>

#include "display_drivers.hpp"

namespace espp {
/**
 * @brief Display driver for the ST7789 display controller.
 *
 *   This code is modified from
 *   https://github.com/lvgl/lvgl_esp32_drivers/blob/master/lvgl_tft/st7789.c
 *   and
 *   https://github.com/Bodmer/TFT_eSPI/blob/master/TFT_Drivers/ST7789_Defines.h
 *
 *   See also:
 *   https://github.com/espressif/esp-who/blob/master/components/screen/controller_driver/st7789/st7789.c
 *   or
 *   https://github.com/espressif/tflite-micro-esp-examples/blob/master/components/screen/controller_driver/st7789/st7789.c
 *   or https://esphome.io/api/st7789v_8h_source.html or
 *   https://github.com/mireq/esp32-st7789-demo/blob/master/components/st7789/include/st7789.h
 *   or
 *   https://github.com/mireq/esp32-st7789-demo/blob/master/components/st7789/st7789.c
 *
 * \section st7789_ttgo_cfg TTGO St7789 Config
 * \snippet display_drivers_example.cpp ttgo_config example
 * \section st7789_box_cfg ESP32-S3-BOX St7789 Config
 * \snippet display_drivers_example.cpp box_config example
 * \section st7789_ex1 st7789 Example
 * \snippet display_drivers_example.cpp display_drivers example
 */
class St7789 {
public:
  enum class Command : uint8_t {
    nop = 0x00,     // no operation
    swreset = 0x01, // software reset
    rddid = 0x04,   // read display id
    rddst = 0x09,   // read display status

    rddpm = 0x0a,      // read display power mode
    rdd_madctl = 0x0b, // read display madctl
    rdd_colmod = 0x0c, // read display pixel format
    rddim = 0x0d,      // read display image mode
    rddsm = 0x0e,      // read display signal mode
    rddsr = 0x0f,      // read display self-diagnostic result (st7789v)

    slpin = 0x10,  // sleep in
    slpout = 0x11, // sleep out
    ptlon = 0x12,  // partial mode on
    noron = 0x13,  // normal display mode on

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

    ptlar = 0x30,
    vscrdef = 0x33, // vertical scrolling definition (st7789v)
    teoff = 0x34,   // tearing effect line off
    teon = 0x35,    // tearing effect line on
    madctl = 0x36,  // memory data access control
    idmoff = 0x38,  // idle mode off
    idmon = 0x39,   // idle mode on
    ramwrc = 0x3c,  // memory write continue (st7789v)
    ramrdc = 0x3e,  // memory read continue (st7789v)
    colmod = 0x3a,  // color mode - pixel format

    ramctrl = 0xb0,   // ram control
    rgbctrl = 0xb1,   // rgb control
    porctrl = 0xb2,   // porch control
    frctrl1 = 0xb3,   // frame rate control
    parctrl = 0xb5,   // partial mode control
    gctrl = 0xb7,     // gate control
    gtadj = 0xb8,     // gate on timing adjustment
    dgmen = 0xba,     // digital gamma enable
    vcoms = 0xbb,     // vcoms setting
    lcmctrl = 0xc0,   // lcm control
    idset = 0xc1,     // id setting
    vdvvrhen = 0xc2,  // vdv and vrh command enable
    vrhs = 0xc3,      // vrh set
    vdvset = 0xc4,    // vdv setting
    vcmofset = 0xc5,  // vcoms offset set
    frctr2 = 0xc6,    // fr control 2
    cabcctrl = 0xc7,  // cabc control
    regsel1 = 0xc8,   // register value section 1
    regsel2 = 0xca,   // register value section 2
    pwmfrsel = 0xcc,  // pwm frequency selection
    pwctrl1 = 0xd0,   // power control 1
    vapvanen = 0xd2,  // enable vap/van signal output
    cmd2en = 0xdf,    // command 2 enable
    pvgamctrl = 0xe0, // positive voltage gamma control
    nvgamctrl = 0xe1, // negative voltage gamma control
    dgmlutr = 0xe2,   // digital gamma look-up table for red
    dgmlutb = 0xe3,   // digital gamma look-up table for blue
    gatectrl = 0xe4,  // gate control
    spi2en = 0xe7,    // spi2 enable
    pwctrl2 = 0xe8,   // power control 2
    eqctrl = 0xe9,    // equalize time control
    promctrl = 0xec,  // program control
    promen = 0xfa,    // program mode enable
    nvmset = 0xfc,    // nvm setting
    promact = 0xfe,   // program action
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

    // set up the init commands
    auto init_commands = std::to_array<display_drivers::DisplayInitCmd<>>({
        {0xCF, {0x00, 0x83, 0X30}},
        {0xED, {0x64, 0x03, 0X12, 0X81}},
        {(uint8_t)Command::pwctrl2, {0x85, 0x01, 0x79}},
        {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}},
        {0xF7, {0x20}},
        {0xEA, {0x00, 0x00}},
        {(uint8_t)Command::lcmctrl, {0x26}},
        {(uint8_t)Command::idset, {0x11}},
        {(uint8_t)Command::vcmofset, {0x35, 0x3E}},
        {(uint8_t)Command::cabcctrl, {0xBE}},
        {(uint8_t)Command::madctl, {madctl}},
        {(uint8_t)Command::colmod, {0x55}},
        {(uint8_t)Command::invon},
        {(uint8_t)Command::rgbctrl, {0x00, 0x1B}},
        {0xF2, {0x08}},
        {(uint8_t)Command::gamset, {0x01}},
        {(uint8_t)Command::caset, {0x00, 0x00, 0x00, 0xEF}},
        {(uint8_t)Command::raset, {0x00, 0x00, 0x01, 0x3f}},
        {(uint8_t)Command::ramwr},
        {(uint8_t)Command::gctrl, {0x07}},
        {0xB6, {0x0A, 0x82, 0x27, 0x00}},
        {(uint8_t)Command::slpout, {0}, 100},
        {(uint8_t)Command::dispon, {0}, 100},
    });

    // NOTE: ST7789 setting the reverse color is the normal color so we inver
    // the logic here.
    if (config.invert_colors) {
      init_commands[12].command = (uint8_t)Command::invoff;
    } else {
      init_commands[12].command = (uint8_t)Command::invon;
    }

    // send the init commands
    send_commands(init_commands);
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
    static uint16_t color_data[max_bytes_to_send];
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
