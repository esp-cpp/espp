#pragma once

#include "driver/gpio.h"
#include "display.hpp"

namespace espp {
  namespace display_drivers {
    struct Config {
      Display::write_fn lcd_write; /**< Function which the display driver uses to write data to the display. */
      gpio_num_t reset_pin; /**< GPIO used for resetting the display. */
      gpio_num_t data_command_pin; /**< GPIO used for indicating to the LCD whether the bits are data or command bits. */
      gpio_num_t backlight_pin; /**< GPIO used for controlling the backlight of the display. */
      bool invert_colors{false}; /**< Whether to invert the colors on the display. */
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

    static void init_pins(gpio_num_t reset, gpio_num_t data_command, gpio_num_t backlight) {
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
      gpio_set_level(backlight, 0);

      using namespace std::chrono_literals;
      //Reset the display
      gpio_set_level(reset, 0);
      std::this_thread::sleep_for(100ms);
      gpio_set_level(reset, 1);
      std::this_thread::sleep_for(100ms);
    }

    static void send_command(uint8_t command, gpio_num_t dc_pin, Display::write_fn lcd_write) {
      gpio_set_level(dc_pin, (uint8_t)Mode::COMMAND);
      lcd_write((uint8_t*)&command, 1, (uint32_t)Display::Signal::NONE);
    }

    static void send_data(uint8_t *data, size_t length, gpio_num_t dc_pin, Display::write_fn lcd_write) {
      gpio_set_level(dc_pin, (uint8_t)Mode::DATA);
      lcd_write(data, length, (uint32_t)Display::Signal::NONE);
    }

    static void send_colors(uint8_t *data, size_t length, gpio_num_t dc_pin, Display::write_fn lcd_write) {
      gpio_set_level(dc_pin, (uint8_t)Mode::DATA);
      lcd_write(data, length, (uint32_t)Display::Signal::FLUSH);
    }

    static void send_commands(LcdInitCmd *commands, gpio_num_t dc_pin, Display::write_fn lcd_write) {
      using namespace std::chrono_literals;
      //Send all the commands
      uint16_t cmd = 0;
      while (commands[cmd].length!=0xff) {
        send_command(commands[cmd].command, dc_pin, lcd_write);
        send_data(commands[cmd].data, commands[cmd].length&0x1F, dc_pin, lcd_write);
        if (commands[cmd].length & 0x80) {
          std::this_thread::sleep_for(100ms);
        }
        cmd++;
      }
    }
  } // namespace display_drivers
} // namespace espp
