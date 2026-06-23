#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <thread>
#include <utility>
#include <vector>

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
 * @brief SPI/parallel/DSI panel transport interface for object-style display
 *        controllers.
 */
class PanelIo {
public:
  /// @brief Virtual destructor.
  virtual ~PanelIo() = default;

  /// @brief Check whether the transport has been initialized successfully.
  /// @return True if the transport is ready for use.
  virtual bool initialized() const = 0;

  /// @brief Send a command and optional parameter payload immediately.
  /// @param command Command byte to transmit.
  /// @param parameters Optional command payload bytes.
  /// @param flags Optional transport-specific user flags.
  virtual void write_command(uint8_t command, std::span<const uint8_t> parameters,
                             uint32_t flags = 0) = 0;

  /// @brief Queue a command byte for asynchronous transmission.
  /// @param command Command byte to transmit.
  /// @param flags Optional transport-specific user flags.
  virtual void queue_command(uint8_t command, uint32_t flags = 0) = 0;

  /// @brief Queue a non-pixel data payload for asynchronous transmission.
  /// @param data Payload bytes to transmit.
  /// @param flags Optional transport-specific user flags.
  virtual void queue_data(std::span<const uint8_t> data, uint32_t flags = 0) = 0;

  /// @brief Queue a pixel payload for asynchronous transmission.
  /// @param data Pointer to the pixel payload.
  /// @param size Payload size in bytes.
  /// @param flags Optional transport-specific user flags.
  /// @param transaction_flags Optional low-level transaction flags.
  virtual void queue_pixels(const uint8_t *data, size_t size, uint32_t flags = 0,
                            uint32_t transaction_flags = 0) = 0;

  /// @brief Wait for all queued transfers to complete.
  virtual void wait() = 0;
};

/**
 * @brief Config structure for all display drivers.
 */
struct Config {
  PanelIo *panel_io{nullptr};     /**< Optional object-style transport used by the controller. */
  write_command_fn write_command; /**< Legacy low-level function used by the display driver to write
                                       commands to the display. */
  read_command_fn read_command{
      nullptr}; /**< Legacy low-level function used by the display driver to read commands from
                     the display. Optional, may be nullptr if not supported. */
  send_lines_fn lcd_send_lines; /**< Legacy low-level function used by the display driver to send
                                     bulk color data asynchronously. Optional, may be nullptr. */
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
 * @brief Common rectangle used by the reusable display-driver base classes.
 */
struct Region {
  int xs{0};
  int ys{0};
  int xe{0};
  int ye{0};
};

/**
 * @brief Initialize the display pins.
 * @param reset GPIO pin used for resetting the display.
 * @param data_command GPIO pin used for indicating to the LCD whether the bits are data or command
 * @param reset_value The value to set the reset pin to when resetting the display.
 */
inline void init_pins(gpio_num_t reset, gpio_num_t data_command, uint8_t reset_value) {
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

/**
 * @brief Build a controller-specific MADCTL base value from the shared config.
 * @param config Display configuration.
 * @param color_order_bit Bit used to swap RGB/BGR ordering.
 * @param mirror_x_bit Bit used to mirror the X axis.
 * @param mirror_y_bit Bit used to mirror the Y axis.
 * @param swap_xy_bit Bit used to swap X/Y addressing, or 0 if unsupported.
 * @return Base MADCTL value before runtime rotation is applied.
 */
inline uint8_t make_madctl_base(const Config &config, uint8_t color_order_bit, uint8_t mirror_x_bit,
                                uint8_t mirror_y_bit, uint8_t swap_xy_bit) {
  uint8_t value = 0;
  if (config.swap_color_order) {
    value |= color_order_bit;
  }
  if (config.mirror_x) {
    value |= mirror_x_bit;
  }
  if (config.mirror_y) {
    value |= mirror_y_bit;
  }
  if (config.swap_xy && swap_xy_bit != 0) {
    value |= swap_xy_bit;
  }
  return value;
}

/**
 * @brief Apply the standard four-orientation transform to a MADCTL value.
 * @param value Base MADCTL value.
 * @param config Display configuration.
 * @param rotation Desired display rotation.
 * @param mirror_x_bit Bit used to mirror the X axis.
 * @param mirror_y_bit Bit used to mirror the Y axis.
 * @param swap_xy_bit Bit used to swap X/Y addressing, or 0 if unsupported.
 * @return Rotated MADCTL value.
 */
inline uint8_t apply_standard_rotation(uint8_t value, const Config &config,
                                       DisplayRotation rotation, uint8_t mirror_x_bit,
                                       uint8_t mirror_y_bit, uint8_t swap_xy_bit) {
  switch (rotation) {
  case DisplayRotation::LANDSCAPE:
    break;
  case DisplayRotation::PORTRAIT:
    if (config.mirror_portrait) {
      value ^= (mirror_x_bit | swap_xy_bit);
    } else {
      value ^= (mirror_y_bit | swap_xy_bit);
    }
    break;
  case DisplayRotation::LANDSCAPE_INVERTED:
    value ^= (mirror_y_bit | mirror_x_bit);
    break;
  case DisplayRotation::PORTRAIT_INVERTED:
    if (config.mirror_portrait) {
      value ^= (mirror_y_bit | swap_xy_bit);
    } else {
      value ^= (mirror_x_bit | swap_xy_bit);
    }
    break;
  }
  return value;
}

/**
 * @brief Object-style base class for display controllers.
 */
class Controller {
public:
  /// @brief Construct a controller from the shared display configuration.
  /// @param config Display configuration.
  explicit Controller(const Config &config)
      : config_(config) {}

  /// @brief Virtual destructor.
  virtual ~Controller() = default;

  /// @brief Initialize the concrete display controller.
  /// @return True on success.
  virtual bool initialize() = 0;

  /// @brief Update the controller rotation state.
  /// @param rotation New display rotation.
  virtual void set_rotation(const DisplayRotation &rotation) { rotation_ = rotation; }

  /// @brief Write an area of pixel data without implicitly notifying LVGL.
  /// @param disp LVGL display pointer, used when synchronous flush completion is needed.
  /// @param area Area to update.
  /// @param color_map Pixel buffer for the area.
  /// @param flags Transport/user flags passed through to the write path.
  virtual void fill(lv_display_t *disp, const lv_area_t *area, uint8_t *color_map,
                    uint32_t flags = 0) {
    auto pixel_count = lv_area_get_width(area) * lv_area_get_height(area);
    preprocess_color_map(color_map, pixel_count);
    write_region({.xs = area->x1, .ys = area->y1, .xe = area->x2, .ye = area->y2}, color_map,
                 pixel_count * bytes_per_pixel(), flags);
    if ((flags & flush_flag()) != 0 && !uses_async_flush() && disp != nullptr) {
      lv_display_flush_ready(disp);
    }
  }

  /// @brief Flush an area of pixel data and notify LVGL when appropriate.
  /// @param disp LVGL display pointer.
  /// @param area Area to flush.
  /// @param color_map Pixel buffer for the area.
  virtual void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *color_map) {
    fill(disp, area, color_map, flush_flag());
  }

  /// @brief Clear a rectangular region to a solid color.
  /// @param x Starting X coordinate.
  /// @param y Starting Y coordinate.
  /// @param width Width in pixels.
  /// @param height Height in pixels.
  /// @param color RGB565 color value.
  virtual void clear(size_t x, size_t y, size_t width, size_t height, uint16_t color = 0x0000) = 0;

  /// @brief Update the controller's logical top-left pixel offset.
  /// @param x X offset in pixels.
  /// @param y Y offset in pixels.
  virtual void set_offset(int x, int y) {
    config_.offset_x = x;
    config_.offset_y = y;
  }

  /// @brief Read the controller's logical top-left pixel offset.
  /// @param x Filled with the current X offset.
  /// @param y Filled with the current Y offset.
  virtual void get_offset(int &x, int &y) const {
    x = config_.offset_x;
    y = config_.offset_y;
  }

protected:
  /// @brief Flag used to signal LVGL flush completion through low-level transports.
  /// @return Bitmask containing the flush flag.
  static constexpr uint32_t flush_flag() {
    return 1u << static_cast<uint32_t>(display_drivers::Flags::FLUSH_BIT);
  }

  /// @brief Get bytes-per-pixel from the configured bit depth.
  /// @return Number of bytes per pixel.
  size_t bytes_per_pixel() const { return std::max<size_t>(1, config_.bits_per_pixel / 8); }

  /// @brief Check whether this controller is using a `PanelIo` transport.
  /// @return True if `panel_io` is configured and initialized.
  bool uses_panel_io() const {
    return config_.panel_io != nullptr && config_.panel_io->initialized();
  }

  /// @brief Check whether the active transport completes flushes asynchronously.
  /// @return True if the transport signals completion later.
  bool uses_async_flush() const { return uses_panel_io() || config_.lcd_send_lines != nullptr; }

  /// @brief Get the configured X/Y offset after applying the current rotation convention.
  /// @return Rotated `(x, y)` offset pair.
  std::pair<int, int> get_rotated_offset() const {
    switch (rotation_) {
    case DisplayRotation::PORTRAIT:
    case DisplayRotation::PORTRAIT_INVERTED:
      return {config_.offset_y, config_.offset_x};
    case DisplayRotation::LANDSCAPE:
    case DisplayRotation::LANDSCAPE_INVERTED:
    default:
      return {config_.offset_x, config_.offset_y};
    }
  }

  /// @brief Apply the rotated offset to a region.
  /// @param region Region in logical display coordinates.
  /// @return Offset-adjusted region.
  Region apply_offset(Region region) const {
    auto [offset_x, offset_y] = get_rotated_offset();
    region.xs += offset_x;
    region.xe += offset_x;
    region.ys += offset_y;
    region.ye += offset_y;
    return region;
  }

  /// @brief Allow concrete controllers to transform a region before writing it.
  /// @param region Region after offsets have been applied.
  /// @return Controller-specific transformed region.
  virtual Region transform_region(Region region) const { return region; }

  /// @brief Send a list of initialization commands.
  /// @tparam Command Command enum or byte type.
  /// @param commands Sequence of initialization commands.
  template <typename Command>
  void send_commands(std::span<const DisplayInitCmd<Command>> commands) {
    using namespace std::chrono_literals;
    for (const auto &[command, parameters, delay_ms] : commands) {
      write_command(static_cast<uint8_t>(command), parameters, 0);
      if (delay_ms > 0) {
        std::this_thread::sleep_for(delay_ms * 1ms);
      }
    }
  }

  /// @brief Send a fixed-size array of initialization commands.
  /// @tparam Command Command enum or byte type.
  /// @tparam N Number of commands.
  /// @param commands Sequence of initialization commands.
  template <typename Command, size_t N>
  void send_commands(const std::array<DisplayInitCmd<Command>, N> &commands) {
    send_commands(std::span<const DisplayInitCmd<Command>>(commands.data(), commands.size()));
  }

  /// @brief Write a command through the active transport.
  /// @param command Command byte.
  /// @param parameters Optional payload bytes.
  /// @param flags Optional transport/user flags.
  void write_command(uint8_t command, std::span<const uint8_t> parameters = {},
                     uint32_t flags = 0) {
    if (uses_panel_io()) {
      config_.panel_io->write_command(command, parameters, flags);
      return;
    }
    if (config_.write_command) {
      config_.write_command(command, parameters, flags);
    }
  }

  /// @brief Read command data through the legacy callback path.
  /// @param command Command byte.
  /// @param data Buffer to fill with returned data.
  /// @param flags Optional transport/user flags.
  void read_command(uint8_t command, std::span<uint8_t> data, uint32_t flags = 0) {
    if (config_.read_command) {
      config_.read_command(command, data, flags);
    }
  }

  /// @brief Perform any controller-specific preprocessing on a pixel buffer before writeout.
  /// @param color_map Pixel buffer.
  /// @param pixel_count Number of pixels in the buffer.
  virtual void preprocess_color_map(uint8_t *color_map, size_t pixel_count) const {
    if (config_.bits_per_pixel == 16) {
      lv_draw_sw_rgb565_swap(color_map, pixel_count);
    }
  }

  /// @brief Write a transformed region to the active transport.
  /// @param region Region to write.
  /// @param data Pixel payload bytes.
  /// @param size Payload size in bytes.
  /// @param flags Optional transport/user flags.
  virtual void write_region(Region region, const uint8_t *data, size_t size, uint32_t flags) = 0;

  Config config_;
  std::mutex io_mutex_;
  DisplayRotation rotation_{DisplayRotation::LANDSCAPE};
};

/**
 * @brief Shared base for command/data/window-based MIPI DBI panels.
 */
class MipiDbiDisplayDriver : public Controller {
public:
  /// @brief DBI protocol commands used by a concrete controller.
  struct Protocol {
    uint8_t column_address_command{0}; ///< CASET-equivalent command.
    uint8_t row_address_command{0};    ///< RASET-equivalent command.
    uint8_t memory_write_command{0};   ///< RAMWR-equivalent command.
    bool use_8bit_coordinates{false};  ///< Whether region coordinates are encoded as 8-bit values.
  };

  /// @brief Construct a shared MIPI DBI-style controller base.
  /// @param config Shared display configuration.
  /// @param protocol Controller-specific command mapping.
  explicit MipiDbiDisplayDriver(const Config &config, const Protocol &protocol)
      : Controller(config)
      , protocol_(protocol) {}

  /// @brief Clear a rectangular region to a solid color.
  /// @param x Starting X coordinate.
  /// @param y Starting Y coordinate.
  /// @param width Width in pixels.
  /// @param height Height in pixels.
  /// @param color RGB565 color value.
  void clear(size_t x, size_t y, size_t width, size_t height, uint16_t color = 0x0000) override {
    auto region = transform_region(apply_offset({.xs = static_cast<int>(x),
                                                 .ys = static_cast<int>(y),
                                                 .xe = static_cast<int>(x + width),
                                                 .ye = static_cast<int>(y + height)}));

    std::array<uint16_t, 1024> color_words;
    color_words.fill(color);

    std::scoped_lock lock(io_mutex_);
    if (uses_panel_io()) {
      config_.panel_io->wait();
    }
    write_window(region);

    auto total_pixels = width * height;
    for (size_t written = 0; written < total_pixels; written += color_words.size()) {
      auto chunk_pixels = std::min(total_pixels - written, color_words.size());
      write_command(protocol_.memory_write_command,
                    {reinterpret_cast<const uint8_t *>(color_words.data()), chunk_pixels * 2}, 0);
    }
  }

protected:
  /// @brief Write a pixel region using either `PanelIo`, legacy async callbacks, or blocking
  /// writes.
  /// @param region Region to update.
  /// @param data Pixel payload bytes.
  /// @param size Payload size in bytes.
  /// @param flags Optional transport/user flags.
  void write_region(Region region, const uint8_t *data, size_t size, uint32_t flags) override {
    region = transform_region(apply_offset(region));

    if (uses_panel_io()) {
      std::scoped_lock lock(io_mutex_);
      config_.panel_io->wait();
      queue_window(region);
      config_.panel_io->queue_command(protocol_.memory_write_command);
      config_.panel_io->queue_pixels(data, size, flags);
      return;
    }

    if (config_.lcd_send_lines) {
      config_.lcd_send_lines(region.xs, region.ys, region.xe, region.ye, data, flags);
      return;
    }

    std::scoped_lock lock(io_mutex_);
    write_window(region);
    write_command(protocol_.memory_write_command, {data, size}, flags);
  }

  /// @brief Write the column/row window registers synchronously.
  /// @param region Region to encode as the active drawing window.
  void write_window(const Region &region) {
    auto [column_bytes, column_size] = encode_range(region.xs, region.xe);
    auto [row_bytes, row_size] = encode_range(region.ys, region.ye);
    write_command(protocol_.column_address_command, {column_bytes.data(), column_size}, 0);
    write_command(protocol_.row_address_command, {row_bytes.data(), row_size}, 0);
  }

  /// @brief Queue the column/row window registers for asynchronous transmission.
  /// @param region Region to encode as the active drawing window.
  void queue_window(const Region &region) {
    auto [column_bytes, column_size] = encode_range(region.xs, region.xe);
    auto [row_bytes, row_size] = encode_range(region.ys, region.ye);
    config_.panel_io->queue_command(protocol_.column_address_command);
    config_.panel_io->queue_data({column_bytes.data(), column_size});
    config_.panel_io->queue_command(protocol_.row_address_command);
    config_.panel_io->queue_data({row_bytes.data(), row_size});
  }

  /// @brief Encode a start/end coordinate range for the controller protocol.
  /// @param start Starting coordinate.
  /// @param end Ending coordinate.
  /// @return Encoded byte buffer and number of valid bytes.
  std::pair<std::array<uint8_t, 4>, size_t> encode_range(int start, int end) const {
    std::array<uint8_t, 4> bytes{};
    if (protocol_.use_8bit_coordinates) {
      bytes[0] = start & 0xff;
      bytes[1] = end & 0xff;
      return {bytes, 2};
    }

    bytes[0] = (start >> 8) & 0xff;
    bytes[1] = start & 0xff;
    bytes[2] = (end >> 8) & 0xff;
    bytes[3] = end & 0xff;
    return {bytes, 4};
  }

  Protocol protocol_;
};
} // namespace display_drivers
} // namespace espp

#include "spi_panel_io.hpp"
