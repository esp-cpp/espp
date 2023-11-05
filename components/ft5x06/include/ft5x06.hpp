#pragma once

#include <functional>

#include "logger.hpp"

namespace espp {
/// @brief The FT5x06 touch controller.
/// @details This class is used to communicate with the FT5x06 touch controller.
///
/// \section Example
/// \snippet ft5x06_example.cpp ft5x06 example
class Ft5x06 {
public:
  /// @brief The default I2C address for the FT5x06.
  static constexpr uint8_t DEFAULT_ADDRESS = (0x38);

  /// @brief The function to write data to the I2C bus.
  /// @param address The I2C address of the device.
  /// @param data The data to write to the I2C bus.
  /// @param data_len The length of the data to write to the I2C bus.
  /// @return true if the function succeeds.
  typedef std::function<bool(uint8_t, uint8_t *, size_t)> write_fn;

  /// @brief The function to read data starting at a register from the I2C bus.
  /// @param address The I2C address of the device.
  /// @param register The register to write to the I2C bus.
  /// @param data The data to read from the I2C bus.
  /// @param data_len The length of the data to read from the I2C bus.
  /// @return true if the function succeeds.
  typedef std::function<bool(uint8_t, uint8_t, uint8_t *, size_t)> read_at_register_fn;

  /// @brief The gesture that was detected.
  enum class Gesture : uint8_t {
    NONE = 0x00,
    MOVE_UP = 0x10,
    MOVE_LEFT = 0x14,
    MOVE_DOWN = 0x18,
    MOVE_RIGHT = 0x1C,
    ZOOM_IN = 0x48,
    ZOOM_OUT = 0x49,
  };

  /// @brief The configuration for the FT5x06.
  struct Config {
    write_fn write; ///< The function to write data to the I2C bus.
    read_at_register_fn
        read_at_register; ///< The function to write then read data from the I2C bus.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< The log level.
  };

  /// @brief Construct a new FT5x06.
  /// @param config The configuration for the FT5x06.
  explicit Ft5x06(const Config &config)
      : write_(config.write), read_at_register_(config.read_at_register),
        logger_({.tag = "Ft5x06", .level = config.log_level}) {
    std::error_code ec;
    init(ec);
    if (ec) {
      logger_.error("Failed to initialize FT5x06");
    }
  }

  /// @brief Get the number of touch points.
  /// @param ec The error code if the function fails.
  /// @return The number of touch points.
  uint8_t get_num_touch_points(std::error_code &ec) {
    return read_register(Registers::TOUCH_POINTS, ec);
  }

  /// @brief Get the touch point.
  /// @param num_touch_points The number of touch points.
  /// @param x The x coordinate of the touch point.
  /// @param y The y coordinate of the touch point.
  /// @param ec The error code if the function fails.
  void get_touch_point(uint8_t *num_touch_points, uint16_t *x, uint16_t *y, std::error_code &ec) {
    auto tp = get_num_touch_points(ec);
    if (ec) {
      return;
    }
    *num_touch_points = tp;
    if (*num_touch_points != 0) {
      uint8_t data[4];
      read(Registers::TOUCH1_XH, data, 4, ec);
      if (ec) {
        return;
      }
      *x = ((data[0] & 0x0f) << 8) + data[1];
      *y = ((data[2] & 0x0f) << 8) + data[3];
      logger_.info("Got touch ({}, {})", *x, *y);
    }
  }

  /// @brief Get the gesture that was detected.
  /// @param ec The error code if the function fails.
  /// @return The gesture that was detected.
  Gesture read_gesture(std::error_code &ec) {
    return (Gesture)read_register(Registers::GESTURE_ID, ec);
  }

protected:
  void init(std::error_code &ec) {
    // Valid touching detect threshold
    write_register(Registers::ID_G_THGROUP, 70, ec);
    if (ec)
      return;
    // valid touching peak detect threshold
    write_register(Registers::ID_G_THPEAK, 60, ec);
    if (ec)
      return;
    // Touch focus threshold
    write_register(Registers::ID_G_THCAL, 16, ec);
    if (ec)
      return;
    // threshold when there is surface water
    write_register(Registers::ID_G_THWATER, 60, ec);
    if (ec)
      return;
    // threshold of temperature compensation
    write_register(Registers::ID_G_THTEMP, 10, ec);
    if (ec)
      return;
    // Touch difference threshold
    write_register(Registers::ID_G_THDIFF, 20, ec);
    if (ec)
      return;
    // Delay to enter 'Monitor' status (s)
    write_register(Registers::ID_G_TIME_ENTER_MONITOR, 2, ec);
    if (ec)
      return;
    // Period of 'Active' status (ms)
    write_register(Registers::ID_G_PERIODACTIVE, 12, ec);
    if (ec)
      return;
    // Timer to enter 'idle' when in 'Monitor' (ms)
    write_register(Registers::ID_G_PERIODMONITOR, 40, ec);
  }

  enum class Registers : uint8_t {
    DEVICE_MODE = 0x00,
    GESTURE_ID = 0x01,
    TOUCH_POINTS = 0x02,

    TOUCH1_EV_FLAG = 0x03,
    TOUCH1_XH = 0x03,
    TOUCH1_XL = 0x04,
    TOUCH1_YH = 0x05,
    TOUCH1_YL = 0x06,

    TOUCH2_EV_FLAG = 0x09,
    TOUCH2_XH = 0x09,
    TOUCH2_XL = 0x0A,
    TOUCH2_YH = 0x0B,
    TOUCH2_YL = 0x0C,

    TOUCH3_EV_FLAG = 0x0F,
    TOUCH3_XH = 0x0F,
    TOUCH3_XL = 0x10,
    TOUCH3_YH = 0x11,
    TOUCH3_YL = 0x12,

    TOUCH4_EV_FLAG = 0x15,
    TOUCH4_XH = 0x15,
    TOUCH4_XL = 0x16,
    TOUCH4_YH = 0x17,
    TOUCH4_YL = 0x18,

    TOUCH5_EV_FLAG = 0x1B,
    TOUCH5_XH = 0x1B,
    TOUCH5_XL = 0x1C,
    TOUCH5_YH = 0x1D,
    TOUCH5_YL = 0x1E,

    ID_G_THGROUP = 0x80,
    ID_G_THPEAK = 0x81,
    ID_G_THCAL = 0x82,
    ID_G_THWATER = 0x83,
    ID_G_THTEMP = 0x84,
    ID_G_THDIFF = 0x85,
    ID_G_CTRL = 0x86,
    ID_G_TIME_ENTER_MONITOR = 0x87,
    ID_G_PERIODACTIVE = 0x88,
    ID_G_PERIODMONITOR = 0x89,
    ID_G_AUTO_CLB_MODE = 0xA0,
    ID_G_LIB_VERSION_H = 0xA1,
    ID_G_LIB_VERSION_L = 0xA2,
    ID_G_CIPHER = 0xA3,
    ID_G_MODE = 0xA4,
    ID_G_PMODE = 0xA5,
    ID_G_FIRMID = 0xA6,
    ID_G_STATE = 0xA7,
    ID_G_FT5201ID = 0xA8,
    ID_G_ERR = 0xA9,
  };

  uint8_t read_register(Registers reg, std::error_code &ec) {
    uint8_t data = 0;
    bool success = read_at_register_(DEFAULT_ADDRESS, (uint8_t)reg, &data, 1);
    if (!success) {
      logger_.error("Failed to read register {}", (uint8_t)reg);
      ec = std::make_error_code(std::errc::io_error);
      return 0;
    }
    return data;
  }

  void read(Registers reg, uint8_t *data, size_t data_len, std::error_code &ec) {
    bool success = read_at_register_(DEFAULT_ADDRESS, (uint8_t)reg, data, data_len);
    if (!success) {
      logger_.error("Failed to read register {}", (uint8_t)reg);
      ec = std::make_error_code(std::errc::io_error);
    }
  }

  void write_register(Registers reg, uint8_t data, std::error_code &ec) {
    uint8_t buf[2] = {(uint8_t)reg, data};
    bool success = write_(DEFAULT_ADDRESS, buf, 2);
    if (!success) {
      logger_.error("Failed to write register {}", (uint8_t)reg);
      ec = std::make_error_code(std::errc::io_error);
    }
  }

  write_fn write_;
  read_at_register_fn read_at_register_;
  espp::Logger logger_;
};
} // namespace espp
