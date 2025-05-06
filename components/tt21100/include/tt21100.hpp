#pragma once

#include <functional>

#include "base_peripheral.hpp"

namespace espp {
/// @brief Driver for the Tt21100 touch controller
///
/// \section tt21100_ex1 Example
/// \snippet tt21100_example.cpp tt21100 example
class Tt21100 : public BasePeripheral<> {
public:
  /// @brief The default i2c address
  static constexpr uint8_t DEFAULT_ADDRESS = (0x24);

  /// @brief Configuration for the Tt21100 driver
  struct Config {
    BasePeripheral::write_fn write = nullptr; ///< Function for writing to the i2c device (unused)
    BasePeripheral::read_fn read;             ///< Function for reading from the i2c device
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log level
  };

  /// @brief Constructor
  /// @param config The configuration for the driver
  explicit Tt21100(const Config &config)
      : BasePeripheral({.address = DEFAULT_ADDRESS, .read = config.read}, "Tt21100",
                       config.log_level) {
    std::error_code ec;
    init(ec);
    if (ec) {
      logger_.error("Failed to initialize: {}", ec.message());
    }
  }

  /// @brief Read the touch data
  /// @param ec The error code to set if an error occurs
  /// @return True if there is new touch data
  bool update(std::error_code &ec) {
    static uint16_t data_len;
    static uint8_t data[256];

    // NOTE: this chip is weird, and even though we're reading a u16, we can't
    //       use the read_u16 since that function assumes the data is in little
    //       endian format, but this chip sends the data in big endian format meaning
    //       the bytes are swapped

    read_many((uint8_t *)&data_len, 2, ec);
    if (ec) {
      logger_.error("Failed to read data length: {}", ec.message());
      return false;
    }

    logger_.debug("Data length: {}", data_len);

    if (data_len >= 0xff) {
      logger_.error("Invalid data length");
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }

    read_many(data, data_len, ec);
    if (ec) {
      logger_.error("Failed to read data");
      return false;
    }

    bool new_data = false;
    switch (data_len) {
    case 2:
      // no available data
      break;
    case 7:
    case 17:
    case 27: {
      // touch event - NOTE: this only gets the first touch record
      const auto report_data = (const TouchReport *)data;
      const auto touch_data = (const TouchRecord *)(&report_data->touch_record[0]);
      x_ = touch_data->x;
      y_ = touch_data->y;
      num_touch_points_ = (data_len - sizeof(TouchReport)) / sizeof(TouchRecord);
      logger_.debug("Touch event: #={}, [0]=({}, {})", num_touch_points_, x_, y_);
      new_data = true;
      break;
    }
    case 14: {
      // button event
      const auto button_data = (const ButtonRecord *)data;
      home_button_pressed_ = button_data->btn_val;
      auto btn_signal = button_data->btn_signal[0];
      logger_.debug("Button event({}): {}, {}", (int)(button_data->length), home_button_pressed_,
                    btn_signal);
      new_data = true;
      break;
    }
    default:
      break;
    }
    return new_data;
  }

  /// @brief Get the number of touch points
  /// @note This is the number of touch points that were present when the last
  ///       update() was called
  /// @return The number of touch points
  uint8_t get_num_touch_points() const { return num_touch_points_; }

  /// @brief Get the touch point data
  /// @note This is the touch point data that was present when the last
  ///       update() was called
  /// @param num_touch_points The number of touch points
  /// @param x The x position of the touch point
  /// @param y The y position of the touch point
  void get_touch_point(uint8_t *num_touch_points, uint16_t *x, uint16_t *y) const {
    *num_touch_points = get_num_touch_points();
    if (*num_touch_points != 0) {
      *x = x_;
      *y = y_;
    }
  }

  /// @brief Get the state of the home button
  /// @note This is the state of the home button when the last update() was
  ///       called
  /// @return True if the home button is pressed
  uint8_t get_home_button_state() const { return home_button_pressed_; }

protected:
  void init(std::error_code &ec) {
    logger_.debug("Initializing...");
    uint16_t reg_val = 0;
    static constexpr int max_tries = 10;
    int num_tries = 0;
    do {
      using namespace std::chrono_literals;
      read_many((uint8_t *)&reg_val, 2, ec);
      if (ec) {
        logger_.error("Failed to read...");
        return;
      }
      logger_.debug("reg_val: {:#04x}", reg_val);
      std::this_thread::sleep_for(20ms);
    } while (0x0002 != reg_val && ++num_tries < max_tries);
    if (num_tries >= max_tries) {
      logger_.warn("Reached max tries trying to read...");
    }
  }

  enum class Registers : uint8_t {
    TP_NUM = 0x01,
    X_POS = 0x02,
    Y_POS = 0x03,
  };

  struct TouchRecord {
    uint8_t : 5;
    uint8_t touch_type : 3;
    uint8_t tip : 1;
    uint8_t event_id : 2;
    uint8_t touch_id : 5;
    uint16_t x;
    uint16_t y;
    uint8_t pressure;
    uint16_t major_axis_length;
    uint8_t orientation;
  } __attribute__((packed));

  struct TouchReport {
    uint16_t data_len;
    uint8_t report_id;
    uint16_t time_stamp;
    uint8_t : 2;
    uint8_t large_object : 1;
    uint8_t record_num : 5;
    uint8_t report_counter : 2;
    uint8_t : 3;
    uint8_t noise_efect : 3;
    TouchRecord touch_record[0];
  } __attribute__((packed));

  struct ButtonRecord {
    uint16_t length;     /*!< Always 14(0x000E) */
    uint8_t report_id;   /*!< Always 03h */
    uint16_t time_stamp; /*!< Number in units of 100 us */
    uint8_t btn_val;     /*!< Only use bit[0..3] */
    uint16_t btn_signal[4];
  } __attribute__((packed));

  std::atomic<bool> home_button_pressed_{false};
  std::atomic<uint8_t> num_touch_points_;
  std::atomic<uint16_t> x_;
  std::atomic<uint16_t> y_;
};
} // namespace espp
