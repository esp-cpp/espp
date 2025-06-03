#pragma once

#include <memory>
#include <mutex>

#include "display_drivers.hpp"

namespace espp {
/**
 * @brief Display driver for the SSD1351 display controller.
 *
 *   This code is modified from
 *   https://github.com/adafruit/Adafruit-SSD1351-library/blob/master/Adafruit_SSD1351.cpp
 *   and
 *   https://github.com/lvgl/lvgl_esp32_drivers/blob/master/lvgl_tft/ssd1306.c
 *   and
 *   https://github.com/rdagger/micropython-ssd1351/blob/master/ssd1351.py
 *   and
 *   https://github.com/Bodmer/TFT_eSPI/blob/master/TFT_Drivers/SSD1351_Defines.h
 *
 * \section ssd1351_byte90_cfg Byte90 Ssd1351 Config
 * \snippet display_drivers_example.cpp byte90_config example
 * \section ssd1351_ex1 ssd1351 Example
 * \snippet display_drivers_example.cpp display_drivers example
 */
class Ssd1351 {
public:
  /**
   * @brief Power control settings for the SSD1351 display.
   */
  enum class PowerControl1 : uint8_t {
    // Bits 7-6: Reserved
    // Bits 5-4: Phase 2 period (0x0-0x3)
    PHASE2_1 = 0x00, // 1 DCLK
    PHASE2_2 = 0x10, // 2 DCLK
    PHASE2_3 = 0x20, // 3 DCLK
    PHASE2_4 = 0x30, // 4 DCLK

    // Bits 3-2: Phase 1 period (0x0-0x3)
    PHASE1_1 = 0x00, // 1 DCLK
    PHASE1_2 = 0x04, // 2 DCLK
    PHASE1_3 = 0x08, // 3 DCLK
    PHASE1_4 = 0x0C, // 4 DCLK

    // Bits 1-0: Clock divider ratio (0x0-0x3)
    CLK_DIV_1 = 0x00, // Divide by 1
    CLK_DIV_2 = 0x01, // Divide by 2
    CLK_DIV_4 = 0x02, // Divide by 4
    CLK_DIV_8 = 0x03, // Divide by 8

    // Common configurations
    DEFAULT = 0x32,   // Default power control setting
    LOW_POWER = 0x33, // Low power mode
    HIGH_PERF = 0x30, // High performance mode
  };

  /**
   * @brief Power control settings for the SSD1351 display.
   */
  enum class PowerControl2 : uint8_t {
    // Bits 7-6: Reserved
    // Bits 5-4: VGH voltage level (0x0-0x3)
    VGH_6_6V = 0x00, // 6.6V
    VGH_7_4V = 0x10, // 7.4V
    VGH_8_0V = 0x20, // 8.0V
    VGH_9_0V = 0x30, // 9.0V

    // Bits 3-2: VGL voltage level (0x0-0x3)
    VGL_10_0V = 0x00, // -10.0V
    VGL_9_0V = 0x04,  // -9.0V
    VGL_8_0V = 0x08,  // -8.0V
    VGL_7_0V = 0x0C,  // -7.0V

    // Bits 1-0: Reserved

    // Common configurations
    DEFAULT = 0x01,   // Default power control setting
    LOW_POWER = 0x00, // Low power mode
    HIGH_PERF = 0x30, // High performance mode
  };

  enum class Command : uint8_t {
    nop = 0xE3,        // No operation
    swreset = 0x01,    // Software reset
    rddid = 0x04,      // Read display ID
    rddst = 0x09,      // Read display status
    rddpm = 0x0A,      // Read display power mode
    rdd_madctl = 0x0B, // Read display MADCTL
    rdd_colmod = 0x0C, // Read display pixel format
    rddim = 0x0D,      // Read display image mode
    rddsm = 0x0E,      // Read display signal mode
    rddsr = 0x0F,      // Read display self-diagnostic result

    slpin = 0x10,  // Sleep in
    slpout = 0x11, // Sleep out
    ptlon = 0x12,  // Partial mode on
    noron = 0x13,  // Normal display mode on

    invoff = 0x20,  // Display inversion off
    invon = 0x21,   // Display inversion on
    gamset = 0x26,  // Gamma set
    dispoff = 0x28, // Display off
    dispon = 0x29,  // Display on
    caset = 0x15,   // Column address set
    raset = 0x75,   // Row address set
    ramwr = 0x5C,   // RAM write
    rgbset = 0x2D,  // Color setting
    ramrd = 0x5D,   // RAM read

    ptlar = 0x30,   // Partial area
    vscrdef = 0x33, // Vertical scrolling definition
    teoff = 0x34,   // Tearing effect line off
    teon = 0x35,    // Tearing effect line on
    madctl = 0x36,  // Memory data access control
    idmoff = 0x38,  // Idle mode off
    idmon = 0x39,   // Idle mode on
    ramwrc = 0x3C,  // Memory write continue
    ramrdc = 0x3E,  // Memory read continue
    colmod = 0x3A,  // Color mode - pixel format

    ramctrl = 0xB0,   // RAM control
    rgbctrl = 0xB1,   // RGB control
    porctrl = 0xB2,   // Porch control
    frctrl1 = 0xB3,   // Frame rate control
    parctrl = 0xB5,   // Partial mode control
    gctrl = 0xB7,     // Gate control
    gtadj = 0xB8,     // Gate on timing adjustment
    dgmen = 0xBA,     // Digital gamma enable
    vcoms = 0xBB,     // VCOMH setting
    lcmctrl = 0xC0,   // LCM control
    idset = 0xC1,     // ID setting
    vdvvrhen = 0xC2,  // VDV and VRH command enable
    vrhs = 0xC3,      // VRH set
    vdvset = 0xC4,    // VDV setting
    vcmofset = 0xC5,  // VCOMH offset set
    frctr2 = 0xC6,    // Frame rate control 2
    cabcctrl = 0xC7,  // CABC control
    regsel1 = 0xC8,   // Register value section 1
    regsel2 = 0xCA,   // Register value section 2
    pwmfrsel = 0xCC,  // PWM frequency selection
    pwctrl1 = 0xD0,   // Power control 1
    vapvanen = 0xD2,  // Enable VAP/VAN signal output
    cmd2en = 0xDF,    // Command 2 enable
    pvgamctrl = 0xE0, // Positive voltage gamma control
    nvgamctrl = 0xE1, // Negative voltage gamma control
    dgmlutr = 0xE2,   // Digital gamma look-up table for red
    dgmlutb = 0xE3,   // Digital gamma look-up table for blue
    gatectrl = 0xE4,  // Gate control
    spi2en = 0xE7,    // SPI2 enable
    pwctrl2 = 0xE8,   // Power control 2
    eqctrl = 0xE9,    // Equalize time control
    promctrl = 0xEC,  // Program control
    promen = 0xFA,    // Program mode enable
    nvmset = 0xFC,    // NVM setting
    promact = 0xFE,   // Program action
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
        // Command lock - unlock OLED driver IC MCU interface
        {(uint8_t)Command::cmd2en, {0x12}, 0}, // Unlock commands
        {(uint8_t)Command::cmd2en, {0xB1}, 0}, // Make commands A2,B1,B3,BB,BE,C1 accessible

        // Display off
        {(uint8_t)Command::dispoff, {}, 0},

        // Power control settings
        {(uint8_t)Command::pwctrl1, {static_cast<uint8_t>(PowerControl1::DEFAULT)}, 0},
        {(uint8_t)Command::pwctrl2, {static_cast<uint8_t>(PowerControl2::DEFAULT)}, 0},

        // Clock divider and oscillator frequency
        {(uint8_t)Command::frctrl1, {0xF1}, 0}, // 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio

        // Multiplex ratio (128 lines)
        {(uint8_t)Command::gctrl, {0x7F}, 0}, // 127 (128-1)

        // Set remap and color depth
        {(uint8_t)Command::madctl,
         {
             (config.mirror_x ? 0x40 : 0) | (config.mirror_y ? 0x80 : 0) |
             (config.swap_xy ? 0x20 : 0) | (config.swap_color_order ? 0x08 : 0) |
             0x04 // RGB color order
         },
         0},

        // Color mode setting
        {(uint8_t)Command::colmod, {0x55}, 0}, // Set 16-bit color mode

        // Set column address range
        {(uint8_t)Command::caset, {0x00, 0x7F}, 0}, // Column start/end address

        // Set row address range
        {(uint8_t)Command::raset, {0x00, 0x7F}, 0}, // Row start/end address

        // Set display start line
        {(uint8_t)Command::idset, {0x00}, 0},

        // Set display offset
        {(uint8_t)Command::vdvvrhen, {0x00}, 0},

        // Set GPIO pins
        {(uint8_t)Command::lcmctrl, {0x00}, 0},

        // Function selection (internal VDD regulator)
        {(uint8_t)Command::pwctrl1, {0x01}, 0},

        // Set segment low voltage
        {(uint8_t)Command::vrhs, {0xA0, 0xB5, 0x55}, 0},

        // Set contrast current for colors A, B, C
        {(uint8_t)Command::regsel1, {0xC8}, 0}, // Red
        {(uint8_t)Command::regsel2, {0x80}, 0}, // Green
        {(uint8_t)Command::regsel1, {0xC8}, 0}, // Blue

        // Master current control
        {(uint8_t)Command::cabcctrl, {0x0F}, 0},

        // Set precharge speed for colors A, B, C
        {(uint8_t)Command::porctrl, {0x32, 0x32, 0x32}, 0},

        // Set precharge voltage
        {(uint8_t)Command::vcoms, {0x17}, 0},

        // Set VCOMH voltage
        {(uint8_t)Command::vcmofset, {0x05}, 0},

        // Gamma correction
        {(uint8_t)Command::pvgamctrl,
         {0x3F, 0x25, 0x1C, 0x1E, 0x20, 0x12, 0x2A, 0x90, 0x24, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00},
         0},
        {(uint8_t)Command::nvgamctrl,
         {0x3F, 0x20, 0x43, 0x45, 0x33, 0x16, 0x08, 0x09, 0x06, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00},
         0},

        // Normal or inverted display
        {config.invert_colors ? (uint8_t)Command::invoff : (uint8_t)Command::invon, {}, 0},

        // Clear display memory
        {(uint8_t)Command::caset, {0x00, 0x7F}, 0},
        {(uint8_t)Command::raset, {0x00, 0x7F}, 0},
        {(uint8_t)Command::ramwr, {}, 0},

        // Display on
        {(uint8_t)Command::dispon, {}, 0},
    });

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

  /**
   * @brief Set the power control settings for the display.
   * @param pc1 Power control 1 settings
   * @param pc2 Power control 2 settings
   */
  static void set_power_control(PowerControl1 pc1, PowerControl2 pc2) {
    std::scoped_lock lock{spi_mutex_};
    write_command_(static_cast<uint8_t>(Command::pwctrl1), {static_cast<uint8_t>(pc1)}, 0);
    write_command_(static_cast<uint8_t>(Command::pwctrl2), {static_cast<uint8_t>(pc2)}, 0);
  }

  /**
   * @brief Get the current power control settings.
   * @param pc1 Reference to store power control 1 settings
   * @param pc2 Reference to store power control 2 settings
   */
  static void get_power_control(PowerControl1 &pc1, PowerControl2 &pc2) {
    std::scoped_lock lock{spi_mutex_};
    uint8_t data[1];

    // Read power control 1
    write_command_(static_cast<uint8_t>(Command::rddid), data, 0);
    pc1 = static_cast<PowerControl1>(data[0]);

    // Read power control 2
    write_command_(static_cast<uint8_t>(Command::rddid), data, 0);
    pc2 = static_cast<PowerControl2>(data[0]);
  }

  /**
   * @brief Set the display to low power mode.
   */
  static void set_low_power_mode() {
    set_power_control(PowerControl1::LOW_POWER, PowerControl2::LOW_POWER);
  }

  /**
   * @brief Set the display to high performance mode.
   */
  static void set_high_performance_mode() {
    set_power_control(PowerControl1::HIGH_PERF, PowerControl2::HIGH_PERF);
  }

  /**
   * @brief Set the display to default power mode.
   */
  static void set_default_power_mode() {
    set_power_control(PowerControl1::DEFAULT, PowerControl2::DEFAULT);
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
