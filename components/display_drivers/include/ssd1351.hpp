#pragma once

#include <memory>
#include <mutex>

#include "display_drivers.hpp"

namespace espp {
/**
 * @brief Display driver for the SSD1351 128x128 RGB OLED display controller.
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
 *   Datasheet can be found here:
 *   https://cdn-shop.adafruit.com/datasheets/SSD1351-Revision+1.3.pdf
 *
 * \section ssd1351_byte90_cfg Byte90 Ssd1351 Config
 * \snippet display_drivers_example.cpp byte90_config example
 * \section ssd1351_ex1 ssd1351 Example
 * \snippet display_drivers_example.cpp display_drivers example
 */
class Ssd1351 {
public:
  /**
   * @brief Enum for Command values used by the SSD1351 display controller.
   */
  enum class Command : uint8_t {
    caset = 0x15, ///< Column address set
    raset = 0x75, ///< Row address set
    ramwr = 0x5C, ///< RAM write
    ramrd = 0x5D, ///< RAM read

    madctl = 0xA0, ///< Memory data access control

    STARTLINE = 0xA1, ///< Vertical Scroll by RAM

    DISPLAYOFFSET = 0xA2, ///< Vertical Scroll by row (locked)
    DISPLAYALLOFF = 0xA4, ///< All pixels off
    DISPLAYALLON = 0xA5,  ///< All pixels on (all pixels have GS63)
    NORMALDISPLAY = 0xA6, ///< Normal display mode
    INVERTDISPLAY = 0xA7, ///< Inverted display mode

    FUNCTIONSELECT = 0xAB, ///< See datasheet

    DISPLAYOFF = 0xAE, ///< Sleep Mode On
    DISPLAYON = 0xAF,  ///< Sleep Mode Off

    PRECHARGE = 0xB1,      ///< See datasheet
    DISPLAYENHANCE = 0xB2, ///< Not currently used
    CLOCKDIV = 0xB3,       ///< See datasheet
    SETVSL = 0xB4,         ///< See datasheet
    SETGPIO = 0xB5,        ///< See datasheet
    PRECHARGE2 = 0xB6,     ///< See datasheet
    SETGRAY = 0xB8,        ///< Not currently used
    USELUT = 0xB9,         ///< Not currently used
    PRECHARGELEVEL = 0xBB, ///< Not currently used
    VCOMH = 0xBE,          ///< See datasheet
    CONTRASTABC = 0xC1,    ///< See datasheet
    CONTRASTMASTER = 0xC7, ///< See datasheet
    MUXRATIO = 0xCA,       ///< See datasheet
    HORIZSCROLL = 0x96,    ///< Not currently used
    STOPSCROLL = 0x9E,     ///< Not currently used
    STARTSCROLL = 0x9F,    ///< Not currently used

    nop0 = 0xD1, ///< No operation (0)
    nop1 = 0xE3, ///< No operation (1)

    COMMANDLOCK = 0xFD, ///< Command lock and MCU protection status
  };

  // madctl bits:
  // 6,7 Color depth (01 = 64K)
  // 5   Odd/even split COM (0: disable, 1: enable) - split COM
  // 4   Scan direction (0: top-down, 1: bottom-up) - mirror Y
  // 3   Reserved - always 0
  // 2   Color remap (0: A->B->C, 1: C->B->A) - BGR color order
  // 1   Column remap (0: 0-127, 1: 127-0) - mirror X
  // 0   Address increment (0: horizontal, 1: vertical) - swap X and Y
  static constexpr int OLED_CMD_COLOR_DEPTH_65K1 = 0b00 << 6;  // 65K color mode
  static constexpr int OLED_CMD_COLOR_DEPTH_65K2 = 0b01 << 6;  // 65K color mode
  static constexpr int OLED_CMD_COLOR_DEPTH_262K1 = 0b10 << 6; // 262K color mode
  static constexpr int OLED_CMD_COLOR_DEPTH_262K2 = 0b11 << 6; // 262K color mode, 16-bit format 2
  static constexpr int OLED_CMD_COM_SPLIT = 0b1 << 5;          // Odd/even split COM
  static constexpr int OLED_CMD_MY_BIT = 0b1 << 4;             // Mirror Y
  static constexpr int OLED_CMD_BGR_BIT = 0b1 << 2;            // BGR color order
  static constexpr int OLED_CMD_MX_BIT = 0b1 << 1;             // Mirror X
  static constexpr int OLED_CMD_MV_BIT = 0b1 << 0;             // Swap X and Y

  static constexpr int DEFAULT_MADCTL =
      OLED_CMD_COLOR_DEPTH_65K2 | OLED_CMD_COM_SPLIT; ///< Default MADCTL value

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

    uint8_t madctl = DEFAULT_MADCTL; // Default MADCTL value
    if (swap_color_order_) {
      madctl |= OLED_CMD_BGR_BIT;
    }
    if (mirror_x_) {
      madctl |= OLED_CMD_MX_BIT;
    }
    if (mirror_y_) {
      madctl |= OLED_CMD_MY_BIT;
    }
    if (swap_xy_) {
      madctl |= OLED_CMD_MV_BIT;
    }

    // set up the init commands
    auto init_commands = std::to_array<display_drivers::DisplayInitCmd<>>({
        // Command lock - unlock OLED driver IC MCU interface
        {(uint8_t)Command::COMMANDLOCK, {0x12}, 0}, // Unlock commands
        {(uint8_t)Command::COMMANDLOCK, {0xB1}, 0}, // Make commands A2,B1,B3,BB,BE,C1 accessible

        // Display off
        {(uint8_t)Command::DISPLAYOFF, {}, 0},

        // Clock divider and oscillator frequency
        {(uint8_t)Command::CLOCKDIV, {0xF1}, 0}, // 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio

        // Multiplex ratio (128 lines)
        {(uint8_t)Command::MUXRATIO, {0x7F}, 0}, // 127 (128-1)

        {(uint8_t)Command::DISPLAYOFFSET, {0x00}, 0},  // Set display offset to 0
        {(uint8_t)Command::SETGPIO, {0x00}, 0},        // Set GPIO to 0 (disable)
        {(uint8_t)Command::FUNCTIONSELECT, {0x01}, 0}, // Internal VDD regulator
        {(uint8_t)Command::PRECHARGE, {0x32}, 0},      // Set precharge speed
        {(uint8_t)Command::VCOMH, {0x05}, 0},          // Set VCOMH voltage

        // // Normal or inverted display
        {config.invert_colors ? (uint8_t)Command::INVERTDISPLAY : (uint8_t)Command::NORMALDISPLAY,
         {},
         0},

        {(uint8_t)Command::CONTRASTABC, {0xC8, 0x80, 0xC8}, 0}, // Set contrast for colors A, B, C
        {(uint8_t)Command::CONTRASTMASTER, {0x0F}, 0},          // Set master contrast
        {(uint8_t)Command::SETVSL, {0xA0, 0xB5, 0x55}, 0},      // Set VSL voltage
        {(uint8_t)Command::PRECHARGE2, {0x01}, 0},              // Set precharge 2 speed
        {(uint8_t)Command::PRECHARGELEVEL, {0x1C, 0x1C, 0x1C}, 0}, // Set precharge level

        // Display on
        {(uint8_t)Command::DISPLAYON, {}, 0},

        // Set remap and color depth
        {(uint8_t)Command::madctl, {madctl}, 0},

        // Set display start line
        {(uint8_t)Command::STARTLINE, {0x00}, 0},
    });

    // send the init commands
    send_commands(init_commands);
  }

  /**
   * @brief Set the display rotation.
   * @param rotation New display rotation.
   */
  static void rotate(const DisplayRotation &rotation) {
    uint8_t madctl = DEFAULT_MADCTL; // Default MADCTL value
    uint8_t startline = 0;
    if (swap_color_order_) {
      madctl |= OLED_CMD_BGR_BIT;
    }
    if (mirror_x_) {
      madctl |= OLED_CMD_MX_BIT;
    }
    if (mirror_y_) {
      madctl |= OLED_CMD_MY_BIT;
    }
    if (swap_xy_) {
      madctl |= OLED_CMD_MV_BIT;
    }
    switch (rotation) {
    case DisplayRotation::LANDSCAPE: {
      // set startline to HEIGHT
      startline = 127;
      std::scoped_lock lock{spi_mutex_};
      write_command_(static_cast<uint8_t>(Command::STARTLINE), {&startline, 1}, 0);
    } break;
    case DisplayRotation::PORTRAIT: {
      // flip the mv bit (xor)
      madctl ^= (OLED_CMD_MV_BIT);
      if (mirror_portrait_) {
        // flip the my bit (xor)
        madctl ^= (OLED_CMD_MY_BIT);
      } else {
        // flip the mx bit (xor)
        madctl ^= OLED_CMD_MX_BIT;
      }
      // set startline to WIDTH
      startline = 127;
      std::scoped_lock lock{spi_mutex_};
      write_command_(static_cast<uint8_t>(Command::STARTLINE), {&startline, 1}, 0);
    } break;
    case DisplayRotation::LANDSCAPE_INVERTED: {
      // flip the my and mx bits (xor)
      madctl ^= (OLED_CMD_MY_BIT | OLED_CMD_MX_BIT);
      // set startline to 0
      startline = 0;
      std::scoped_lock lock{spi_mutex_};
      write_command_(static_cast<uint8_t>(Command::STARTLINE), {&startline, 1}, 0);
    } break;
    case DisplayRotation::PORTRAIT_INVERTED: {
      // flip the mv bit (xor)
      madctl ^= (OLED_CMD_MV_BIT);
      if (mirror_portrait_) {
        // flip the mx bit (xor)
        madctl ^= OLED_CMD_MX_BIT;
      } else {
        // flip the my bit (xor)
        madctl ^= (OLED_CMD_MY_BIT);
      }
      // set startline to 0
      startline = 0;
      std::scoped_lock lock{spi_mutex_};
      write_command_(static_cast<uint8_t>(Command::STARTLINE), {&startline, 1}, 0);
    } break;
    }
    auto madctl_data = std::array<uint8_t, 1>{madctl};
    std::scoped_lock lock{spi_mutex_};
    write_command_(static_cast<uint8_t>(Command::madctl), madctl_data, 0);
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
    std::array<uint8_t, 2> data;

    int offset_x = 0;
    int offset_y = 0;
    get_offset_rotated(offset_x, offset_y);

    uint16_t start_x = xs + offset_x;
    uint16_t end_x = xe + offset_x;
    uint16_t start_y = ys + offset_y;
    uint16_t end_y = ye + offset_y;

    // Set the column (x) start / end addresses
    data[0] = start_x & 0xFF;
    data[1] = end_x & 0xFF;
    write_command_(static_cast<uint8_t>(Command::caset), data, 0);

    // Set the row (y) start / end addresses
    data[0] = start_y & 0xFF;
    data[1] = end_y & 0xFF;
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
      if (delay_ms) {
        std::this_thread::sleep_for(delay_ms * 1ms);
      }
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
   * @brief Set the display brightness.
   * @param brightness Brightness value in range [0.0, 1.0].
   */
  static void set_brightness(const float brightness) {
    // Update the local brightness value
    brightness_ = brightness;

    // This display has a 4-bit brightness control
    uint8_t data = brightness * 15.0f; // Scale to 0-15 range
    write_command_(static_cast<uint8_t>(Command::CONTRASTMASTER), {&data, 1}, 0);
  }

  /**
   * @brief Get the current display brightness.
   * @return Current brightness value in range [0.0, 1.0].
   */
  static float get_brightness() { return brightness_; }

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
  static inline float brightness_ = 1.0f;
};
} // namespace espp
