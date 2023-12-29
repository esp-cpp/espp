#pragma once

#include <functional>

#include "logger.hpp"

namespace espp {
/// @brief Driver for the Tt21100 touch controller
///
/// \section Example
/// \snippet tt21100_example.cpp tt21100 example
class Tt21100 {
public:
  /// @brief The default i2c address
  static constexpr uint8_t DEFAULT_ADDRESS = (0x24);

  /// @brief Function signature for reading from the i2c device
  /// @param dev_addr The device address
  /// @param data The data to read
  /// @param data_len The length of the data to read
  typedef std::function<bool(uint8_t, uint8_t *, size_t)> read_fn;

  /// @brief Configuration for the Tt21100 driver
  struct Config {
    read_fn read; ///< Function for reading from the i2c device
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log level
  };

  /// @brief Constructor
  /// @param config The configuration for the driver
  explicit Tt21100(const Config &config)
      : read_(config.read), logger_({.tag = "Tt21100", .level = config.log_level}) {
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

    bool success = read_(DEFAULT_ADDRESS, (uint8_t *)&data_len, sizeof(data_len));
    if (!success) {
      logger_.error("Failed to read data length");
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }

    logger_.debug("Data length: {}", data_len);

    if (data_len == 0xff) {
      logger_.error("Invalid data length");
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }

    success = read_(DEFAULT_ADDRESS, data, data_len);
    if (!success) {
      logger_.error("Failed to read data");
      ec = std::make_error_code(std::errc::io_error);
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
      auto report_data = (TouchReport *)data;
      const auto touch_data = (TouchRecord *)(&report_data->touch_record[0]);
      x_ = touch_data->x;
      y_ = touch_data->y;
      num_touch_points_ = (data_len - sizeof(TouchReport)) / sizeof(TouchRecord);
      logger_.debug("Touch event: #={}, [0]=({}, {})", num_touch_points_, x_, y_);
      new_data = true;
      break;
    }
    case 14: {
      // button event
      auto button_data = (ButtonRecord *)data;
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
    do {
      using namespace std::chrono_literals;
      bool success = read_(DEFAULT_ADDRESS, (uint8_t *)&reg_val, 2);
      if (!success) {
        logger_.error("Failed to read...");
        ec = std::make_error_code(std::errc::io_error);
        return;
      }
      logger_.debug("reg_val: {:#04x}", reg_val);
      std::this_thread::sleep_for(20ms);
    } while (0x0002 != reg_val);
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

  read_fn read_;
  std::atomic<bool> home_button_pressed_{false};
  std::atomic<uint8_t> num_touch_points_;
  std::atomic<uint16_t> x_;
  std::atomic<uint16_t> y_;
  espp::Logger logger_;
};
} // namespace espp
