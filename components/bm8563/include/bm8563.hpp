#pragma once

#include <ctime>
#include <functional>

#include "base_peripheral.hpp"
#include "espp_chrono.hpp"

namespace espp {
/// @brief The BM8563 RTC driver.
/// @details This driver is for the BM8563 RTC chip.
///
/// \section bm8563_ex1 Example
/// \snippet bm8563_example.cpp bm8563 example
class Bm8563 : public BasePeripheral<> {
public:
  /// @brief The default I2C address for the BM8563.
  static constexpr uint8_t DEFAULT_ADDRESS = (0x51);

  /// @brief The date structure.
  struct Date {
    uint16_t year;   ///< The year.
    uint8_t month;   ///< The month.
    uint8_t weekday; ///< The day of the week.
    uint8_t day;     ///< The day of the month.

    /// @brief Equality operator.
    /// @param other The other date.
    /// @return True if equal, false otherwise.
    bool operator==(const Date &other) const {
      return year == other.year && month == other.month && weekday == other.weekday &&
             day == other.day;
    }

    /// @brief Inequality operator.
    /// @param other The other date.
    /// @return True if not equal, false otherwise.
    bool operator!=(const Date &other) const { return !(*this == other); }
  };

  /// @brief The time structure.
  struct Time {
    uint8_t hour;   ///< The hour.
    uint8_t minute; ///< The minute.
    uint8_t second; ///< The second.

    /// @brief Equality operator.
    /// @param other The other time.
    /// @return True if equal, false otherwise.
    bool operator==(const Time &other) const {
      return hour == other.hour && minute == other.minute && second == other.second;
    }

    /// @brief Inequality operator.
    /// @param other The other time.
    /// @return True if not equal, false otherwise.
    bool operator!=(const Time &other) const { return !(*this == other); }
  };

  /// @brief The date and time structure.
  struct DateTime {
    Date date; ///< The date.
    Time time; ///< The time.

    /// @brief Equality operator.
    /// @param other The other date and time.
    /// @return True if equal, false otherwise.
    bool operator==(const DateTime &other) const {
      return date == other.date && time == other.time;
    }

    /// @brief Inequality operator.
    /// @param other The other date and time.
    /// @return True if not equal, false otherwise.
    bool operator!=(const DateTime &other) const { return !(*this == other); }
  };

  /// @brief The configuration structure.
  struct Config {
    BasePeripheral<>::write_fn write;                     ///< The I2C write function.
    BasePeripheral<>::write_then_read_fn write_then_read; ///< The I2C write then read function.
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; ///< Log verbosity for the input driver.
  };

  /// @brief Constructor.
  /// @param config The configuration.
  explicit Bm8563(const Config &config)
      : BasePeripheral({.address = DEFAULT_ADDRESS,
                        .write = config.write,
                        .write_then_read = config.write_then_read},
                       "Bm8563", config.log_level) {
    std::error_code ec;
    init(ec);
    if (ec) {
      logger_.error("failed to initialize");
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // std::tm based APIs (recommended for portability)
  /////////////////////////////////////////////////////////////////////////////

  /// @brief Set the time using std::tm structure
  /// @param time The time to set
  /// @param ec The error code to set if an error occurs
  /// @return True if the time was set successfully, false otherwise
  bool set_time(const std::tm &time, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    // Validate time values
    if (time.tm_year < 0 || time.tm_year > 199 || // Years 1900-2099
        time.tm_mon < 0 || time.tm_mon > 11 || time.tm_mday < 1 || time.tm_mday > 31 ||
        time.tm_hour < 0 || time.tm_hour > 23 || time.tm_min < 0 || time.tm_min > 59 ||
        time.tm_sec < 0 || time.tm_sec > 59) {
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }

    // Convert std::tm to device-specific format and set
    DateTime dt;
    dt.date.year = time.tm_year + 1900;
    dt.date.month = time.tm_mon + 1;
    dt.date.weekday = time.tm_wday;
    dt.date.day = time.tm_mday;
    dt.time.hour = time.tm_hour;
    dt.time.minute = time.tm_min;
    dt.time.second = time.tm_sec;

    set_date_time(dt, ec);
    return !ec;
  }

  /// @brief Get the time using std::tm structure
  /// @param ec The error code to set if an error occurs
  /// @return The time from the device
  std::tm get_time(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    auto dt = get_date_time(ec);
    if (ec) {
      return {};
    }

    std::tm time = {};
    time.tm_sec = dt.time.second;
    time.tm_min = dt.time.minute;
    time.tm_hour = dt.time.hour;
    time.tm_mday = dt.date.day;
    time.tm_mon = dt.date.month - 1;
    time.tm_year = dt.date.year - 1900;
    time.tm_wday = dt.date.weekday;
    time.tm_yday = -1;  // Let mktime calculate it
    time.tm_isdst = -1; // Let system determine DST

    // Normalize the time structure
    mktime(&time);

    return time;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Device-specific APIs (for backward compatibility)
  /////////////////////////////////////////////////////////////////////////////

  /// @brief Get the date and time (device-specific format).
  /// @param ec The error code.
  /// @return The date and time.
  DateTime get_date_time(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    DateTime dt;
    dt.time = get_time_device(ec);
    if (ec)
      return {};
    dt.date = get_date(ec);
    if (ec)
      return {};
    return dt;
  }

  /// @brief Set the date and time (device-specific format).
  /// @param dt The date and time.
  /// @param ec The error code.
  void set_date_time(const DateTime &dt, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    set_date(dt.date, ec);
    if (ec)
      return;
    set_time_device(dt.time, ec);
  }

  /// @brief Get the date (device-specific format).
  /// @param ec The error code.
  /// @return The date.
  Date get_date(std::error_code &ec) {
    logger_.info("getting date");
    uint8_t data[4];
    read_many_from_register((uint8_t)Registers::DATE, data, 4, ec);
    if (ec) {
      return {};
    }
    Date d;
    int base_year = (data[2] & CENTURY_BIT) ? 1900 : 2000;
    d.year = base_year + bcd_to_decimal(data[3] & 0xff);
    d.month = bcd_to_decimal(data[2] & 0x1f);
    d.weekday = bcd_to_decimal(data[1] & 0x07);
    d.day = bcd_to_decimal(data[0] & 0x3f);
    return d;
  }

  /// @brief Set the date (device-specific format).
  /// @param d The date.
  /// @param ec The error code.
  void set_date(const Date &d, std::error_code &ec) {
    logger_.info("setting date");
    const uint8_t data[] = {decimal_to_bcd(d.day), decimal_to_bcd(d.weekday),
                            (uint8_t)(decimal_to_bcd(d.month) | ((d.year < 2000) ? 0x80 : 0x00)),
                            decimal_to_bcd(d.year % 100)};
    write_many_to_register((uint8_t)Registers::DATE, data, 4, ec);
  }

  /// @brief Get the time (device-specific format).
  /// @param ec The error code.
  /// @return The time.
  Time get_time_device(std::error_code &ec) {
    logger_.info("getting time");
    uint8_t data[3];
    read_many_from_register((uint8_t)Registers::TIME, data, 3, ec);
    if (ec) {
      return {};
    }
    Time t;
    t.hour = bcd_to_decimal(data[2] & 0x3f);
    t.minute = bcd_to_decimal(data[1] & 0x7f);
    t.second = bcd_to_decimal(data[0] & 0x7f);
    return t;
  }

  /// @brief Set the time (device-specific format).
  /// @param t The time.
  /// @param ec The error code.
  void set_time_device(const Time &t, std::error_code &ec) {
    logger_.info("Setting time");
    const uint8_t data[] = {decimal_to_bcd(t.second), decimal_to_bcd(t.minute),
                            decimal_to_bcd(t.hour)};
    write_many_to_register((uint8_t)Registers::TIME, data, 3, ec);
  }

protected:
  void init(std::error_code &ec) {
    logger_.info("initializing");
    const uint8_t data[] = {0, 0};
    write_many_to_register((uint8_t)Registers::CONTROL_STATUS1, data, 2, ec);
  }

  static constexpr int CENTURY_BIT = 0b10000000;
  enum class Registers : uint8_t {
    CONTROL_STATUS1 = 0x00,
    CONTROL_STATUS2 = 0x01,
    TIME = 0x02, // seconds, minutes, hours
    SECONDS = 0x02,
    MINUTES = 0x03,
    HOURS = 0x04,
    DATE = 0x05, // day, weekday, month, year
    DAY = 0x05,
    WEEKDAY = 0x06,
    MONTH = 0x07,
    YEAR = 0x08,

    MINUTE_ALARM = 0x09,
    HOUR_ALARM = 0x0a,
    DAY_ALARM = 0x0b,
    WEEKDAY_ALARM = 0x0c,

    TIMER_CONTROL = 0x0e,
    TIMER = 0x0f,
  };
};
} // namespace espp

// for allowing easy serialization/printing of the
// espp::Bm8563::Date, espp::Bm8563::Time, and espp::Bm8563::DateTime
template <> struct fmt::formatter<espp::Bm8563::Date> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::Bm8563::Date const &d, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "{{.year = {}, .month = {}, .weekday = {}, .day = {}}}",
                          d.year, d.month, d.weekday, d.day);
  }
};

template <> struct fmt::formatter<espp::Bm8563::Time> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::Bm8563::Time const &t, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "{{.hour = {}, .minute = {}, .second = {}}}", t.hour, t.minute,
                          t.second);
  }
};

template <> struct fmt::formatter<espp::Bm8563::DateTime> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::Bm8563::DateTime const &dt, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "{{.date = {}, .time = {}}}", dt.date, dt.time);
  }
};
