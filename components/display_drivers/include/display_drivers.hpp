#pragma once

#include "driver/gpio.h"
#include "esp_lcd_panel_commands.h"
#include "display.hpp"

namespace espp {
  namespace display_drivers {
    struct Config {
      Display::write_fn lcd_write; /**< Function which the display driver uses to write data to the display. */
      gpio_num_t reset_pin; /**< GPIO used for resetting the display. */
      gpio_num_t data_command_pin; /**< GPIO used for indicating to the LCD whether the bits are data or command bits. */
      gpio_num_t backlight_pin; /**< GPIO used for controlling the backlight of the display. */
      bool backlight_on_value{false}; /**< Whether backlight is active high or active low(default). */
      bool invert_colors{false}; /**< Whether to invert the colors on the display. */
      int offset_x{0}; /**< X Gap / offset, in pixels. */
      int offset_y{0}; /**< Y Gap / offset, in pixels. */
      bool swap_xy{false};  /**< Swap row/column order. */
      bool mirror_x{false}; /**< Mirror the display horizontally. */
      bool mirror_y{false}; /**< Mirror the display vertically. */
    };

    /**
     * @brief Mode for configuring the data/command pin.
     */
    enum class Mode { COMMAND = 0, DATA = 1 };

    /**
     * @brief command structure for initializing the lcd
     */
    struct LcdInitCmd {
      uint8_t command; /**< Command byte */
      uint8_t data[16]; /**< Data bytes */
      uint8_t length; /**< Number of data bytes; bit 7 means delay after, 0xFF means end of commands. */
    };

    static void init_pins(gpio_num_t reset, gpio_num_t data_command, gpio_num_t backlight, uint8_t backlight_on) {
      // Initialize display pins
      uint64_t gpio_output_pin_sel =
        ((1ULL << data_command) | (1ULL << reset) | (1ULL << backlight));

      gpio_config_t o_conf{.pin_bit_mask = gpio_output_pin_sel,
                           .mode = GPIO_MODE_OUTPUT,
                           .pull_up_en = GPIO_PULLUP_DISABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};
      ESP_ERROR_CHECK(gpio_config(&o_conf));

      // turn on the backlight
      gpio_set_level(backlight, backlight_on);

      using namespace std::chrono_literals;
      //Reset the display
      gpio_set_level(reset, 0);
      std::this_thread::sleep_for(100ms);
      gpio_set_level(reset, 1);
      std::this_thread::sleep_for(100ms);
    }
  } // namespace display_drivers
} // namespace espp
