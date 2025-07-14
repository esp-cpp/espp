#pragma once

#include "base_peripheral.hpp"
#include "espp_chrono.hpp"

namespace espp {
/// @brief Class for the PCF85063 real-time clock.
/// @details The PCF85063 is a CMOS Real-Time Clock (RTC) and calendar
///          optimized for low power consumption.
/// @note The PCF85063 can be interfaced with I2C.
/// @see https://www.nxp.com/docs/en/data-sheet/PCF85063A.pdf
///
/// @section pcf85063_ex1 Example
/// @snippet pcf85063_example.cpp pcf85063_example
class Pcf85063 : public espp::BasePeripheral<uint8_t, true> {
  // Since the BasePeripheral is a dependent base class (e.g. its template
  // parameters depend on our template parameters), we need to use the `using`
  // keyword to bring in the functions / members we want to use, otherwise we
  // have to either use `this->` or explicitly scope each call, which clutters
  // the code / is annoying. This is needed because of the two phases of name
  // lookups for templates.
  using BasePeripheral<uint8_t, true>::set_address;
  using BasePeripheral<uint8_t, true>::set_write;
  using BasePeripheral<uint8_t, true>::set_read;
  using BasePeripheral<uint8_t, true>::write_u8_to_register;
  using BasePeripheral<uint8_t, true>::write_many_to_register;
  using BasePeripheral<uint8_t, true>::read_u8_from_register;
  using BasePeripheral<uint8_t, true>::read_many_from_register;
  using BasePeripheral<uint8_t, true>::clear_bits_in_register;
  using BasePeripheral<uint8_t, true>::set_bits_in_register;
  using BasePeripheral<uint8_t, true>::base_mutex_;
  using BasePeripheral<uint8_t, true>::logger_;

public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x51; ///< Default I2C address of the PCF85063

  /// @brief The different clock out frequencies that can be used.
  enum class ClockOutFrequency : uint8_t {
    FREQ_32768_HZ = 0, ///< 32.768 kHz
    FREQ_16384_HZ = 1, ///< 16.384 kHz
    FREQ_8192_HZ = 2,  ///< 8.192 kHz
    FREQ_4096_HZ = 3,  ///< 4.096 kHz
    FREQ_2048_HZ = 4,  ///< 2.048 kHz
    FREQ_1024_HZ = 5,  ///< 1.024 kHz
    FREQ_1_HZ = 6,     ///< 1 Hz
    DISABLED = 7,      ///< Disabled
  };

  /// @brief Timer source clock frequency.
  enum class TimerSourceClock : uint8_t {
    FREQ_4096_HZ = 0, ///< 4.096 kHz
    FREQ_64_HZ = 1,   ///< 64 Hz
    FREQ_1_HZ = 2,    ///< 1 Hz
    FREQ_1_60_HZ = 3, ///< 1/60 Hz
  };

  /// @brief Configuration structure for the Pcf85063.
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address of the device.
    write_fn write;                           ///< Write function for the I2C bus.
    read_fn read;                             ///< Read function for the I2C bus.
    bool auto_init{true};                     ///< Whether to automatically initialize the device.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log level for the device.
  };

  /// @brief Construct a new Pcf85063 object
  /// @param config The configuration for the device.
  explicit Pcf85063(const Config &config)
      : BasePeripheral<uint8_t, true>({}, "Pcf85063", config.log_level) {
    set_address(config.device_address);
    set_write(config.write);
    set_read(config.read);
    if (config.auto_init) {
      std::error_code ec;
      init(ec);
      if (ec) {
        logger_.error("Failed to initialize: {}", ec.message());
      }
    }
  }

  /// @brief Initialize the device.
  /// @param ec The error code to set if an error occurs.
  /// @return True if the device was initialized successfully, false otherwise.
  bool init(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // clear the stop bit
    return set_stop(false, ec);
  }

  /// @brief Set the time on the device.
  /// @param time The time to set.
  /// @param ec The error code to set if an error occurs.
  /// @return True if the time was set successfully, false otherwise.
  bool set_time(const std::tm &time, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data[7];
    data[0] = espp::decimal_to_bcd(time.tm_sec);
    data[1] = espp::decimal_to_bcd(time.tm_min);
    data[2] = espp::decimal_to_bcd(time.tm_hour);
    data[3] = espp::decimal_to_bcd(time.tm_mday);
    data[4] = espp::decimal_to_bcd(time.tm_wday);
    data[5] = espp::decimal_to_bcd(time.tm_mon + 1);
    data[6] = espp::decimal_to_bcd(time.tm_year % 100);
    write_many_to_register(static_cast<uint8_t>(Register::SECONDS), data, 7, ec);
    return !ec;
  }

  /// @brief Get the time from the device.
  /// @param ec The error code to set if an error occurs.
  /// @return The time from the device.
  std::tm get_time(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data[7];
    read_many_from_register(static_cast<uint8_t>(Register::SECONDS), data, 7, ec);
    if (ec) {
      return {};
    }
    std::tm time;
    time.tm_sec = espp::bcd_to_decimal(data[0] & 0x7F);
    time.tm_min = espp::bcd_to_decimal(data[1] & 0x7F);
    time.tm_hour = espp::bcd_to_decimal(data[2] & 0x3F);
    time.tm_mday = espp::bcd_to_decimal(data[3] & 0x3F);
    time.tm_wday = espp::bcd_to_decimal(data[4] & 0x07);
    time.tm_mon = espp::bcd_to_decimal(data[5] & 0x1F) - 1;
    time.tm_year = espp::bcd_to_decimal(data[6]) + 100; // tm_year is years since 1900
    // check oscillator stop bit
    if (data[0] & 0x80) {
      logger_.warn("Oscillator has stopped, time may be invalid.");
      ec = std::make_error_code(std::errc::io_error);
    }
    return time;
  }

  /// @brief Set the minute alarm.
  /// @param minute The minute to set the alarm to.
  /// @param enable Whether to enable the alarm.
  /// @param ec The error code to set if an error occurs.
  /// @return True if the alarm was set successfully, false otherwise.
  bool set_minute_alarm(uint8_t minute, bool enable, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = espp::decimal_to_bcd(minute);
    if (!enable) {
      data |= 0x80;
    }
    write_u8_to_register(static_cast<uint8_t>(Register::MINUTE_ALARM), data, ec);
    return !ec;
  }

  /// @brief Set the hour alarm.
  /// @param hour The hour to set the alarm to.
  /// @param enable Whether to enable the alarm.
  /// @param ec The error code to set if an error occurs.
  /// @return True if the alarm was set successfully, false otherwise.
  bool set_hour_alarm(uint8_t hour, bool enable, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = espp::decimal_to_bcd(hour);
    if (!enable) {
      data |= 0x80;
    }
    write_u8_to_register(static_cast<uint8_t>(Register::HOUR_ALARM), data, ec);
    return !ec;
  }

  /// @brief Set the day alarm.
  /// @param day The day to set the alarm to.
  /// @param enable Whether to enable the alarm.
  /// @param ec The error code to set if an error occurs.
  /// @return True if the alarm was set successfully, false otherwise.
  bool set_day_alarm(uint8_t day, bool enable, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = espp::decimal_to_bcd(day);
    if (!enable) {
      data |= 0x80;
    }
    write_u8_to_register(static_cast<uint8_t>(Register::DAY_ALARM), data, ec);
    return !ec;
  }

  /// @brief Set the weekday alarm.
  /// @param weekday The weekday to set the alarm to.
  /// @param enable Whether to enable the alarm.
  /// @param ec The error code to set if an error occurs.
  /// @return True if the alarm was set successfully, false otherwise.
  bool set_weekday_alarm(uint8_t weekday, bool enable, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = espp::decimal_to_bcd(weekday);
    if (!enable) {
      data |= 0x80;
    }
    write_u8_to_register(static_cast<uint8_t>(Register::WEEKDAY_ALARM), data, ec);
    return !ec;
  }

  /// @brief Check if the alarm is triggered.
  /// @param ec The error code to set if an error occurs.
  /// @return True if the alarm is triggered, false otherwise.
  bool is_alarm_triggered(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = read_u8_from_register(static_cast<uint8_t>(Register::CONTROL_2), ec);
    if (ec) {
      return false;
    }
    return data & 0x40;
  }

  /// @brief Clear the alarm flag.
  /// @param ec The error code to set if an error occurs.
  /// @return True if the alarm was cleared successfully, false otherwise.
  bool clear_alarm(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    clear_bits_in_register(static_cast<uint8_t>(Register::CONTROL_2), 0x40, ec);
    return !ec;
  }

  /// @brief Enable the timer.
  /// @param enable Whether to enable the timer.
  /// @param ec The error code to set if an error occurs.
  /// @return True if the timer was enabled/disabled successfully, false otherwise.
  bool enable_timer(bool enable, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (enable) {
      set_bits_in_register(static_cast<uint8_t>(Register::TIMER_MODE), 0x04, ec);
    } else {
      clear_bits_in_register(static_cast<uint8_t>(Register::TIMER_MODE), 0x04, ec);
    }
    return !ec;
  }

  /// @brief Set the timer value.
  /// @param value The value to set the timer to.
  /// @param ec The error code to set if an error occurs.
  /// @return True if the timer was set successfully, false otherwise.
  bool set_timer_value(uint8_t value, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    write_u8_to_register(static_cast<uint8_t>(Register::TIMER_VALUE), value, ec);
    return !ec;
  }

  /// @brief Set the timer clock frequency.
  /// @param freq The frequency to set the timer clock to.
  /// @param ec The error code to set if an error occurs.
  /// @return True if the timer clock was set successfully, false otherwise.
  bool set_timer_clock(TimerSourceClock freq, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = static_cast<uint8_t>(freq);
    set_bits_in_register(static_cast<uint8_t>(Register::TIMER_MODE), data, ec);
    return !ec;
  }

  /// @brief Set the clock out frequency.
  /// @param freq The frequency to set the clock out to.
  /// @param ec The error code to set if an error occurs.
  /// @return True if the clock out was set successfully, false otherwise.
  bool set_clock_out_frequency(ClockOutFrequency freq, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = static_cast<uint8_t>(freq);
    set_bits_in_register(static_cast<uint8_t>(Register::CONTROL_2), data, ec);
    return !ec;
  }

protected:
  /// @brief Set the stop bit.
  /// @param stop Whether to stop the clock.
  /// @param ec The error code to set if an error occurs.
  /// @return True if the stop bit was set successfully, false otherwise.
  bool set_stop(bool stop, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (stop) {
      set_bits_in_register(static_cast<uint8_t>(Register::CONTROL_1), 0x20, ec);
    } else {
      clear_bits_in_register(static_cast<uint8_t>(Register::CONTROL_1), 0x20, ec);
    }
    return !ec;
  }

  enum class Register : uint8_t {
    CONTROL_1 = 0x00,
    CONTROL_2 = 0x01,
    OFFSET = 0x02,
    RAM_BYTE = 0x03,
    SECONDS = 0x04,
    MINUTES = 0x05,
    HOURS = 0x06,
    DAYS = 0x07,
    WEEKDAYS = 0x08,
    MONTHS = 0x09,
    YEARS = 0x0A,
    SECOND_ALARM = 0x0B,
    MINUTE_ALARM = 0x0C,
    HOUR_ALARM = 0x0D,
    DAY_ALARM = 0x0E,
    WEEKDAY_ALARM = 0x0F,
    TIMER_VALUE = 0x10,
    TIMER_MODE = 0x11,
  };
};
} // namespace espp
