#pragma once

#include <functional>

#include "logger.hpp"

namespace espp {
/// @brief A class to interface with the Qwiic NES controller.
/// @details This class is used to interface with the Qwiic NES controller.
///         The Qwiic NES controller is a breakout board for the NES
///         controller.
///         The Qwiic NES controller uses the I2C bus to communicate.
class QwiicNes {
public:
  /// @brief The default I2C address of the device.
  static constexpr uint8_t DEFAULT_ADDRESS = (0x54);

  /// @brief The function to write data to the I2C bus.
  /// @param address The I2C address of the device.
  /// @param data The data to write to the I2C bus.
  /// @param data_len The length of the data to write to the I2C bus.
  /// @return true if the function succeeds.
  typedef std::function<bool(uint8_t, uint8_t *, size_t)> write_fn;

  /// @brief The function to write then read data from the I2C bus.
  /// @param address The I2C address of the device.
  /// @param register The register to write to the I2C bus.
  /// @param data The data to read from the I2C bus.
  /// @param data_len The length of the data to read from the I2C bus.
  /// @return true if the function succeeds.
  typedef std::function<bool(uint8_t, uint8_t, uint8_t *, size_t)> write_read_fn;

  /// @brief The buttons on the NES controller.
  /// @deftails The values in this enum match the button's corresponding bit
  ///           field in the byte returned by read_current_state() and
  ///           read_button_accumulator().
  enum class Button : int { A, B, SELECT, START, UP, DOWN, LEFT, RIGHT };

  /// @brief The state of the buttons on the NES controller.
  /// @details Contains the state of each button on the NES controller.
  struct ButtonState {
    union {
      struct {
        uint8_t a : 1;
        uint8_t b : 1;
        uint8_t select : 1;
        uint8_t start : 1;
        uint8_t up : 1;
        uint8_t down : 1;
        uint8_t left : 1;
        uint8_t right : 1;
      };
      uint8_t raw;
    };
  };

  /// @brief The configuration for the QwiicNes class.
  struct Config {
    write_fn write;           ///< The function to write data to the I2C bus.
    write_read_fn write_read; ///< The function to write then read data from the I2C bus.
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; ///< The log level for the class.
  };

  /// @brief Construct a new QwiicNes object.
  /// @param config The configuration for the QwiicNes class.
  explicit QwiicNes(const Config &config)
      : write_(config.write), write_read_(config.write_read),
        logger_({.tag = "QwiicNes", .level = config.log_level}) {}

  /// @brief Return true if the given button is pressed.
  /// @param state The byte returned by read_current_state().
  /// @param button The button to check.
  /// @return true if the given button is pressed.
  static bool is_pressed(uint8_t state, Button button) {
    int bit = (int)button;
    return state & (1 << bit);
  }

  /// @brief Return true if the given button is pressed.
  /// @param button The button to check.
  /// @return true if the given button is pressed.
  /// @details This function uses the accumulated button states which are
  ///          updated by the update() function.
  bool is_pressed(Button button) const { return is_pressed(accumulated_states_, button); }

  /// @brief Return the current state of the buttons.
  /// @return The current state of the buttons.
  /// @details This function uses the accumulated button states which are
  ///          updated by the update() function.
  ButtonState get_button_state() const { return button_state_; }

  /// @brief Update the state of the buttons.
  /// @param ec The error code if the function fails.
  /// @details This function reads the current state of the buttons and updates
  ///          the accumulated button states.
  ///          This function should be called periodically to ensure the
  ///          accumulated button states are up to date.
  ///          This function will log an error if it fails to read the current
  ///          state of the buttons.
  ///          The accumulated states represent all buttons which have been
  ///          pressed since the last time this function was called.
  /// @see get_button_state()
  /// @see is_pressed()
  void update(std::error_code &ec) {
    auto buttons = read_button_accumulator(ec);
    if (ec) {
      logger_.error("failed to read button accumulator: {}", ec.message());
      return;
    }
    logger_.info("updated state: {:02X}", buttons);
    accumulated_states_ = buttons;
    button_state_.raw = buttons;
  }

  /// @brief Read the current state of the buttons.
  /// @param ec The error code if the function fails.
  /// @return The current state of the buttons.
  /// @details This function will log an error if it fails to read the current
  ///         state of the buttons.
  ///         This function does not update the accumulated button states.
  ///         To update the accumulated button states, call the update()
  ///         function.
  ///         The current state represents the buttons which are currently
  ///         pressed.
  uint8_t read_current_state(std::error_code &ec) const {
    return read_register(Registers::CURRENT_STATE, ec);
  }

  /// @brief Read the current I2C address of the device.
  /// @param ec The error code if the function fails.
  /// @return The current I2C address of the device.
  /// @details This function will log an error if it fails to read the current
  ///        I2C address of the device.
  ///        The I2C address is a 7-bit value.
  ///        The 7-bit I2C address is shifted left by 1 bit and the least
  ///        significant bit is set to 0.
  ///        For example, if the I2C address is 0x54, then the value returned
  ///        by this function will be 0xA8.
  uint8_t read_address(std::error_code &ec) const { return read_register(Registers::ADDRESS, ec); }

  /// @brief Update the I2C address of the device.
  /// @param new_address The new I2C address of the device.
  /// @param ec The error code if the function fails.
  /// @details This function will log an error if it fails to update the I2C
  ///         address of the device.
  ///         The I2C address is a 7-bit value.
  ///         The 7-bit I2C address is shifted left by 1 bit and the least
  ///         significant bit is set to 0.
  void update_address(uint8_t new_address, std::error_code &ec) {
    uint8_t data[2] = {(uint8_t)Registers::CHANGE_ADDRESS, new_address};
    bool success = write_(address_, data, 2);
    if (!success) {
      logger_.error("failed to update address");
      ec = std::make_error_code(std::errc::io_error);
      return;
    }
    address_ = new_address;
  }

protected:
  enum class Registers : uint8_t {
    CURRENT_STATE = 0x00,
    ACCUMULATOR = 0x01,
    ADDRESS = 0x02,
    CHANGE_ADDRESS = 0x03,
  };

  /// @brief Read the button accumulator.
  /// @param ec The error code if the function fails.
  /// @return The button accumulator.
  /// @details This function will log an error if it fails to read the button
  ///         accumulator.
  ///         The button accumulator represents all buttons which have been
  ///         pressed since the last time the accumulator was read - which is
  ///         the last time the update() function or this function was called.
  uint8_t read_button_accumulator(std::error_code &ec) const {
    return read_register(Registers::ACCUMULATOR, ec);
  }

  uint8_t read_register(Registers reg, std::error_code &ec) const {
    uint8_t data;
    bool success = write_read_(address_, (uint8_t)reg, &data, 1);
    if (!success) {
      ec = std::make_error_code(std::errc::io_error);
      return 0;
    }
    return data;
  }

  write_fn write_;
  write_read_fn write_read_;
  uint8_t accumulated_states_{0};
  ButtonState button_state_;
  uint8_t address_{DEFAULT_ADDRESS};
  espp::Logger logger_;
};
} // namespace espp
