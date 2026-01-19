#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "base_component.hpp"
#include "color.hpp"
#include <esp_heap_caps.h>

namespace espp {
/// \brief Class to control LED strips
/// \details This class is used to control LED strips. It is
/// designed to be used with a write function that can be used to
/// write data to the strip. This allows it to be used with
/// different hardware interfaces (e.g. SPI, I2C, RMT, etc.).
/// \note This class does not handle the clock signal (if any) for the strip.
/// It is assumed that the clock signal is handled by the hardware
/// interface.
/// \note This class does not handle the chip select signal (if any) for the
/// strip. It is assumed that the chip select signal is handled by
/// the hardware interface.
/// \note This class does not handle the power signal (if any) for the strip.
///
/// This class is designed to be used to control various LED strips, which may
/// be using different protocols. The following protocols are supported:
/// - APA102 (via SPI)
/// - WS2812 (via the RMT peripheral)
///
/// \section led_strip_ex1 Example 1: APA102 via SPI
/// \snippet led_strip_example.cpp led strip ex1
class LedStrip : public BaseComponent {
public:
  /// \brief Function to write data to the strip
  /// \details This function is used to write data to the strip. It
  /// is assumed that the function will block until the data has
  /// been written.
  /// \param data Pointer to the data to write
  /// \param length Length of the data to write
  /// \note The data is not guaranteed to be valid after the
  /// function returns.
  /// \note The length of the data is guaranteed to be at least 4
  /// bytes.
  /// \note The data is guaranteed to be 4-byte aligned.
  /// \note The data is guaranteed to be in the order of the bytes
  /// in the strip (i.e. the first byte in the data is the first
  /// byte in the strip).
  typedef std::function<void(const uint8_t *data, size_t length)> write_fn;

  /// \brief Byte order for the LEDs
  enum class ByteOrder {
    RGB, ///< RGB byte order
    GRB, ///< GRB byte order
    BGR, ///< BGR byte order
  };

  /// \brief Start frame for the APA102 protocol
  static const std::vector<uint8_t> APA102_START_FRAME;

  /// \brief Configuration for the LedStrip class
  struct Config {
    size_t num_leds;                      ///< Number of LEDs in the strip
    write_fn write;                       ///< Function to write data to the strip
    bool send_brightness{true};           ///< Whether to use the brightness value for the LEDs
    ByteOrder byte_order{ByteOrder::RGB}; ///< Byte order for the LEDs
    std::vector<uint8_t> start_frame{};   ///< Start frame for the strip. Optional - will be sent
                                          ///< before the first LED if not empty.
    std::vector<uint8_t> end_frame{}; ///< End frame for the strip. Optional - will be sent after
                                      ///< the last LED if not empty.
    bool use_dma{false};              ///< Whether to use DMA-capable memory allocation
    uint32_t dma_allocation_flags{
        MALLOC_CAP_DMA}; ///< DMA allocation flags (if use_dma is true). Defaults to MALLOC_CAP_DMA.
    Logger::Verbosity log_level; ///< Log level for this class
  };

  /// \brief Constructor
  /// \param config Configuration for the LedStrip class
  explicit LedStrip(const Config &config)
      : BaseComponent("LedStrip", config.log_level)
      , num_leds_(config.num_leds)
      , send_brightness_(config.send_brightness)
      , byte_order_(config.byte_order)
      , write_(config.write)
      , use_dma_(config.use_dma) {
    // set the color data size
    pixel_size_ = send_brightness_ ? 4 : 3;
    data_size_ = num_leds_ * pixel_size_ + config.start_frame.size() + config.end_frame.size();

    // Allocate memory based on DMA preference
    if (use_dma_) {
      uint32_t dma_flags = config.dma_allocation_flags;
      if (dma_flags == 0) {
        dma_flags = MALLOC_CAP_DMA;
      }
      data_ = static_cast<uint8_t *>(heap_caps_malloc(data_size_, dma_flags));
      if (!data_) {
        logger_.warn("Failed to allocate DMA memory, falling back to regular malloc");
        data_ = static_cast<uint8_t *>(malloc(data_size_));
      }
    } else {
      data_ = static_cast<uint8_t *>(malloc(data_size_));
    }

    if (!data_) {
      logger_.error("Failed to allocate memory for LED strip data");
      return;
    }

    // copy the start frame
    if (!config.start_frame.empty()) {
      memcpy(data_, config.start_frame.data(), config.start_frame.size());
    }

    // copy the end frame
    if (!config.end_frame.empty()) {
      memcpy(data_ + data_size_ - config.end_frame.size(), config.end_frame.data(),
             config.end_frame.size());
    }

    start_offset_ = config.start_frame.size();
    end_offset_ = config.end_frame.size();
  }

  /// \brief Destructor
  /// \details This function frees the memory allocated for the LED strip data.
  ~LedStrip() {
    if (data_) {
      free(data_);
      data_ = nullptr;
    }
  }

  /// \brief Copy constructor (deleted to prevent double-free)
  LedStrip(const LedStrip &) = delete;

  /// \brief Assignment operator (deleted to prevent double-free)
  LedStrip &operator=(const LedStrip &) = delete;

  /// \brief Move constructor
  LedStrip(LedStrip &&other) noexcept
      : BaseComponent(std::move(other))
      , num_leds_(other.num_leds_)
      , send_brightness_(other.send_brightness_)
      , byte_order_(other.byte_order_)
      , pixel_size_(other.pixel_size_)
      , start_offset_(other.start_offset_)
      , end_offset_(other.end_offset_)
      , data_size_(other.data_size_)
      , data_(other.data_)
      , write_(std::move(other.write_))
      , use_dma_(other.use_dma_) {
    other.data_ = nullptr;
    other.data_size_ = 0;
  }

  /// \brief Move assignment operator
  LedStrip &operator=(LedStrip &&other) noexcept {
    if (this != &other) {
      if (data_) {
        free(data_);
      }

      num_leds_ = other.num_leds_;
      send_brightness_ = other.send_brightness_;
      byte_order_ = other.byte_order_;
      pixel_size_ = other.pixel_size_;
      start_offset_ = other.start_offset_;
      end_offset_ = other.end_offset_;
      data_size_ = other.data_size_;
      data_ = other.data_;
      write_ = std::move(other.write_);
      use_dma_ = other.use_dma_;

      other.data_ = nullptr;
      other.data_size_ = 0;
    }
    return *this;
  }

  /// \brief Get the number of LEDs in the strip
  /// \return Number of LEDs in the strip
  size_t num_leds() const { return num_leds_; }

  /// \brief Get the byte order for the LEDs
  /// \return Byte order for the LEDs
  ByteOrder byte_order() const { return byte_order_; }

  /// \brief Shift the LEDs to the left
  /// \param shift_by Number of LEDs to shift by
  /// \note A negative value for shift_by will shift the LEDs to the right
  void shift_left(int shift_by = 1) {
    if (shift_by == 0)
      return;
    if (shift_by >= num_leds_) {
      logger_.error("Shift by {} is greater than the number of LEDs ({})", shift_by, num_leds_);
      return;
    }
    if (shift_by < 0)
      shift_by += num_leds_;
    std::rotate(data_ + start_offset_, data_ + start_offset_ + pixel_size_ * shift_by,
                data_ + data_size_ - end_offset_);
  }

  /// \brief Shift the LEDs to the right
  /// \param shift_by Number of LEDs to shift by
  /// \note A negative value for shift_by will shift the LEDs to the left
  void shift_right(int shift_by = 1) {
    if (shift_by == 0)
      return;
    if (shift_by >= num_leds_) {
      logger_.error("Shift by {} is greater than the number of LEDs ({})", shift_by, num_leds_);
      return;
    }
    if (shift_by < 0)
      shift_by += num_leds_;
    std::rotate(std::reverse_iterator<uint8_t *>(data_ + data_size_ - end_offset_),
                std::reverse_iterator<uint8_t *>(data_ + data_size_ - end_offset_) +
                    pixel_size_ * shift_by,
                std::reverse_iterator<uint8_t *>(data_ + start_offset_));
  }

  /// \brief Set the color of a single LED
  /// \param index Index of the LED to set
  /// \param hsv Color to set the LED to
  /// \param brightness Brightness of the LED
  /// \note The index is zero-based.
  /// \sa Hsv for more information on the HSV color space
  /// \sa show
  void set_pixel(int index, const Hsv &hsv, float brightness = 1.0f) {
    set_pixel(index, hsv.rgb(), brightness);
  }

  /// \brief Set the color of a single LED
  /// \param index Index of the LED to set
  /// \param rgb Color to set the LED to
  /// \param brightness Brightness of the LED
  /// \note The index is zero-based.
  /// \sa Rgb for more information on the RGB color space
  /// \sa show
  void set_pixel(int index, const Rgb &rgb, float brightness = 1.0f) {
    uint8_t brightness_byte = std::clamp<uint8_t>(brightness * 31.0f, 0, 31);
    uint8_t r, g, b;
    r = std::clamp<uint8_t>(rgb.r * 255.0f, 0, 255);
    g = std::clamp<uint8_t>(rgb.g * 255.0f, 0, 255);
    b = std::clamp<uint8_t>(rgb.b * 255.0f, 0, 255);
    set_pixel(index, r, g, b, brightness_byte);
  }

  /// \brief Set the color of a single LED
  /// \param index Index of the LED to set
  /// \param r Red component of the color to set the LED to [0-255]
  /// \param g Green component of the color to set the LED to [0-255]
  /// \param b Blue component of the color to set the LED to [0-255]
  /// \param brightness Brightness of the LED [0-31]
  /// \note The index is zero-based.
  /// \sa show
  void set_pixel(int index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness = 0b11111) {
    if (index < 0 || index >= num_leds_) {
      logger_.error("set_pixel: index out of range: %d", index);
      return;
    }

    int offset = start_offset_ + index * pixel_size_;
    // set the color in the array
    if (send_brightness_) {
      // set the brightness byte (encoded as 0b111nnnnn where nnnnn is the
      // brightness value), this means the brightness value is 0-31
      data_[offset++] = 0b11100000 | brightness;
    } else {
      // we multiply the brightness by the color value to get the correct
      // brightness
      r = (r * brightness) >> 5;
      g = (g * brightness) >> 5;
      b = (b * brightness) >> 5;
    }
    // ensure the byte order is correct
    switch (byte_order_) {
    case ByteOrder::RGB:
      break;
    case ByteOrder::GRB:
      std::swap(r, g);
      break;
    case ByteOrder::BGR:
      std::swap(r, b);
      break;
    }
    // set the color bytes
    data_[offset++] = r;
    data_[offset++] = g;
    data_[offset++] = b;
  }

  /// \brief Set the color of all the LEDs
  /// \param hsv Color to set the LEDs to
  /// \param brightness Brightness of the LEDs
  /// \note The index is zero-based.
  /// \sa set_pixel
  /// \sa show
  void set_all(const Hsv &hsv, float brightness = 1.0f) { set_all(hsv.rgb(), brightness); }

  /// \brief Set the color of all the LEDs
  /// \param rgb Color to set the LEDs to
  /// \param brightness Brightness of the LEDs
  /// \sa set_pixel
  /// \sa show
  void set_all(const Rgb &rgb, float brightness = 1.0f) {
    uint8_t brightness_byte = std::clamp<uint8_t>(brightness * 255.0f, 0, 255);
    set_all(rgb.r, rgb.g, rgb.b, brightness_byte);
  }

  /// \brief Set the color of all the LEDs
  /// \param r Red component of the color to set the LEDs to
  /// \param g Green component of the color to set the LEDs to
  /// \param b Blue component of the color to set the LEDs to
  /// \param brightness Brightness of the LEDs
  /// \sa set_pixel
  /// \sa show
  void set_all(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness = 0xff) {
    // fill out all the color data
    for (int i = 0; i < num_leds_; i++) {
      set_pixel(i, r, g, b, brightness);
    }
  }

  /// \brief Show the colors on the strip
  /// \details This function writes the colors to the strip. It
  /// should be called after setting the colors of the LEDs.
  /// \note This function blocks until the colors have been written
  /// to the strip.
  /// \sa set_pixel
  /// \sa set_all
  void show() {
    if (!data_) {
      logger_.error("No data allocated for LED strip");
      return;
    }
    logger_.debug("writing data {::02x}", std::span<uint8_t>(data_, data_ + data_size_));
    write_(data_, data_size_);
  }

protected:
  size_t num_leds_;
  bool send_brightness_{true};
  ByteOrder byte_order_{ByteOrder::RGB};
  size_t pixel_size_{3};
  size_t start_offset_{0};
  size_t end_offset_{0};
  size_t data_size_{0};
  uint8_t *data_{nullptr};
  write_fn write_;
  bool use_dma_{false};
};
} // namespace espp
