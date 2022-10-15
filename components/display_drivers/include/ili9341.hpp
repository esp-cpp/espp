#pragma once

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
   * \section ili9341_ex1 ili9341 Example
   * \snippet display_drivers_example.cpp ili9341 example
   */
  class Ili9341 {
  public:
    /**
     * @brief Store the config data and send the initialization commands to the
     *        display controller.
     * @param config display_drivers::Config structure
     */
    static void initialize(const display_drivers::Config& config) {
      // update the static members
      lcd_write_ = config.lcd_write;
      reset_pin_ = config.reset_pin;
      dc_pin_ = config.data_command_pin;
      backlight_pin_ = config.backlight_pin;

      // init the display
      display_drivers::LcdInitCmd ili_init_cmds[]={
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
        {0x36, {0x28}, 1},
        {0x3A, {0x55}, 1},
        {0xB1, {0x00, 0x1B}, 2},
        {0xF2, {0x08}, 1},
        {0x26, {0x01}, 1},
        {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
        {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
        {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
        {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
        {0x2C, {0}, 0},
        {0xB7, {0x07}, 1},
        {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
        {0x11, {0}, 0x80},
        {0x29, {0}, 0x80},
        {0, {0}, 0xff},
      };

      // Initialize display pins
      display_drivers::init_pins(reset_pin_, dc_pin_, backlight_pin_);

      // send the init commands
      display_drivers::send_commands(ili_init_cmds, dc_pin_, lcd_write_);

      // configure the display color configuration
      if (config.invert_colors) {
        display_drivers::send_command(0x21, dc_pin_, lcd_write_);
      } else {
        display_drivers::send_command(0x20, dc_pin_, lcd_write_);
      }
    }

    /**
     * @brief Flush the pixel data for the provided area to the display.
     * @param *drv Pointer to the LVGL display driver.
     * @param *area Pointer to the structure describing the pixel area.
     * @param *color_map Pointer to array of colors to flush to the display.
     */
    static void flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
      uint8_t data[4];

      // Set start and end column addresses
      display_drivers::send_command(0x2A, dc_pin_, lcd_write_);
      data[0] = (area->x1 >> 8) & 0xFF;
      data[1] = area->x1 & 0xFF;
      data[2] = (area->x2 >> 8) & 0xFF;
      data[3] = area->x2 & 0xFF;
      display_drivers::send_data(data, 4, dc_pin_, lcd_write_);

      // Set start and end row addresses
      display_drivers::send_command(0x2B, dc_pin_, lcd_write_);
      data[0] = (area->y1 >> 8) & 0xFF;
      data[1] = area->y1 & 0xFF;
      data[2] = (area->y2 >> 8) & 0xFF;
      data[3] = area->y2 & 0xFF;
      display_drivers::send_data(data, 4, dc_pin_, lcd_write_);

      // Write the color data to the configured section of controller memory
      display_drivers::send_command(0x2C, dc_pin_, lcd_write_);
      uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);
      display_drivers::send_colors((uint8_t*)color_map, size * 2, dc_pin_, lcd_write_);
    }

  protected:

    static Display::write_fn lcd_write_;
    static gpio_num_t reset_pin_;
    static gpio_num_t dc_pin_;
    static gpio_num_t backlight_pin_;
  };
}
