#pragma once

#include <mutex>
#include <thread>

#include <driver/gpio.h>
#include <esp_lcd_panel_commands.h>

#include "display.hpp"

namespace espp {
namespace display_drivers {
/**
 * @brief Low-level callback to write bytes to the display controller.
 * @param command to write
 * @param parameters The command parameters to write
 * @param flags User data associated with this transfer, used for flags.
 */
typedef std::function<void(uint8_t command, std::span<const uint8_t> parameters, uint32_t flags)>
    write_command_fn;

/**
 * @brief Low-level callback to read bytes from the display controller.
 * @param command to read
 * @param data Span to store the read data.
 * @param flags User data associated with this transfer, used for flags.
 */
typedef std::function<void(uint8_t command, std::span<uint8_t> data, uint32_t flags)>
    read_command_fn;

/**
 * @brief Send color data to the display, with optional flags.
 * @param sx The starting x-coordinate of the area to fill.
 * @param sy The starting y-coordinate of the area to fill.
 * @param ex The ending x-coordinate of the area to fill.
 * @param ey The ending y-coordinate of the area to fill.
 * @param color_data Pointer to the color data. Should be at least
 *                   (x_end-x_start)*(y_end-y_start)*2 bytes.
 * @param flags Optional flags to send with the transaction.
 */
typedef std::function<void(int sx, int sy, int ex, int ey, const uint8_t *color_data,
                           uint32_t flags)>
    send_lines_fn;

/**
 * @brief Config structure for all display drivers.
 */
struct Config {
  write_command_fn write_command; /**< Function which the display driver uses to write commands to
                                       the display. */
  read_command_fn read_command{
      nullptr};                 /**< Function which the display driver uses to read commands from
                                     the display. Optional, may be nullptr if not supported. */
  send_lines_fn lcd_send_lines; /**< Function which the display driver uses to send bulk (color)
                                     data (non-blocking) to be written to the display. Optional, may
                                   be nullptr. */
  gpio_num_t reset_pin{GPIO_NUM_NC};        /**< Optional GPIO used for resetting the display. */
  gpio_num_t data_command_pin{GPIO_NUM_NC}; /**< Optional GPIO used for indicating to the LCD
                                  whether the bits are data or command bits. */
  bool reset_value{false}; /**< The value to set the reset pin to when resetting the display (low to
                                reset default). */
  uint8_t bits_per_pixel{16};   /**< How many bits per pixel, e.g. [1, 8, 16, 18, 24, 32]*/
  bool invert_colors{false};    /**< Whether to invert the colors on the display. */
  bool swap_color_order{false}; /**< Whether to swap the color order (RGB/BGR) on the display. */
  int offset_x{0};              /**< X Gap / offset, in pixels. */
  int offset_y{0};              /**< Y Gap / offset, in pixels. */
  bool swap_xy{false};          /**< Swap row/column order. */
  bool mirror_x{false};         /**< Mirror the display horizontally. */
  bool mirror_y{false};         /**< Mirror the display vertically. */
  bool mirror_portrait{false};  /**< Mirror the display in portrait mode. */
};

/**
 * @brief Mode for configuring the data/command pin.
 */
enum class Mode {
  COMMAND = 0, /**< Mode for sending commands to the display. */
  DATA = 1     /**< Mode for sending data (config / color) to the display. */
};

/**
 * @brief Flags that will be used by each display driver to signal to the
 *        low level pre/post callbacks to perform different actions.
 */
enum class Flags {
  FLUSH_BIT = 0, /**< Flag for use with the LVGL subsystem, indicating that the display is ready to
                    be flushed. */
  DC_LEVEL_BIT = 1 /**< Flag for use with the pre-transfer callback to set the data/command pin into
                      the correct level for the upcoming transfer. */
};

/**
 * @brief Command structure for initializing the lcd
 */
template <typename Command = uint8_t> struct DisplayInitCmd {
  Command command;                   /**< Command byte */
  std::vector<uint8_t> parameters{}; /**< Optional command parameters */
  size_t delay_ms = 0;               /**< Delay in milliseconds after sending the command. */
};

/**
 * @brief Initialize the display pins.
 * @param reset GPIO pin used for resetting the display.
 * @param data_command GPIO pin used for indicating to the LCD whether the bits are data or command
 * @param reset_value The value to set the reset pin to when resetting the display.
 */
static void init_pins(gpio_num_t reset, gpio_num_t data_command, uint8_t reset_value) {
  // Initialize display pins
  if (reset == GPIO_NUM_NC && data_command == GPIO_NUM_NC) {
    return;
  }
  uint64_t gpio_output_pin_sel = 0;
  if (data_command != GPIO_NUM_NC) {
    gpio_output_pin_sel |= (1ULL << data_command);
  }
  if (reset != GPIO_NUM_NC) {
    gpio_output_pin_sel |= (1ULL << reset);
  }

  gpio_config_t o_conf {
    .pin_bit_mask = gpio_output_pin_sel, .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
#if SOC_GPIO_SUPPORT_PIN_HYS_FILTER
    .hys_ctrl_mode = GPIO_HYS_SOFT_DISABLE,
#endif
  };
  ESP_ERROR_CHECK(gpio_config(&o_conf));

  using namespace std::chrono_literals;
  if (reset != GPIO_NUM_NC) {
    // Reset the display
    gpio_set_level(reset, reset_value);
    std::this_thread::sleep_for(100ms);
    gpio_set_level(reset, !reset_value);
    std::this_thread::sleep_for(100ms);
  }
}
} // namespace display_drivers
} // namespace espp
