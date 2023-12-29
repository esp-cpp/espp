#pragma once

#include <functional>

#include "logger.hpp"

namespace espp {
/// @brief The BM8563 RTC driver.
/// @details This driver is for the BM8563 RTC chip.
///
/// \section Example
/// \snippet bm8563_example.cpp bm8563 example
class Bm8563 {
public:
  /// @brief The default I2C address for the BM8563.
  static constexpr uint8_t DEFAULT_ADDRESS = (0x51);

  /// @brief Function prototype for the I2C write function.
  /// @param address The I2C address to write to.
  /// @param data The data to write.
  /// @param size The number of bytes to write.
  /// @return True if the write was successful, false otherwise.
  typedef std::function<bool(uint8_t, uint8_t *, size_t)> write_fn;

  /// @brief Function prototype for the I2C read function.
  /// @param address The I2C address to read from.
  /// @param register_address The register address to read from.
  /// @param data The data to read into.
  /// @param size The number of bytes to read.
  /// @return True if the read was successful, false otherwise.
  typedef std::function<bool(uint8_t, uint8_t, uint8_t *, size_t)> read_fn;

  /// @brief The date structure.
  struct Date {
    uint16_t year;   ///< The year.
    uint8_t month;   ///< The month.
    uint8_t weekday; ///< The day of the week.
    uint8_t day;     ///< The day of the month.
  };

  /// @brief The time structure.
  struct Time {
    uint8_t hour;   ///< The hour.
    uint8_t minute; ///< The minute.
    uint8_t second; ///< The second.
  };

  /// @brief The date and time structure.
  struct DateTime {
    Date date; ///< The date.
    Time time; ///< The time.
  };

  /// @brief The configuration structure.
  struct Config {
    write_fn write; ///< The I2C write function.
    read_fn read;   ///< The I2C read function.
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; ///< Log verbosity for the input driver.
  };

  /// @brief Constructor.
  /// @param config The configuration.
  explicit Bm8563(const Config &config)
      : write_(config.write), read_(config.read),
        logger_({.tag = "Bm8563", .level = config.log_level}) {
    std::error_code ec;
    init(ec);
    if (ec) {
      logger_.error("failed to initialize");
    }
  }

  /// @brief Convert a BCD value to a byte.
  /// @param value The BCD value.
  /// @return The byte value.
  static uint8_t bcd2byte(uint8_t value) { return (value >> 4) * 10 + (value & 0x0f); }

  /// @brief Convert a byte value to BCD.
  /// @param value The byte value.
  /// @return The BCD value.
  static uint8_t byte2bcd(uint8_t value) { return ((value / 10) << 4) + value % 10; }

  /// @brief Get the date and time.
  /// @return The date and time.
  DateTime get_date_time(std::error_code &ec) {
    DateTime dt;
    dt.time = get_time(ec);
    if (ec)
      return {};
    dt.date = get_date(ec);
    if (ec)
      return {};
    return dt;
  }

  /// @brief Set the date and time.
  /// @param dt The date and time.
  void set_date_time(const DateTime &dt, std::error_code &ec) {
    set_date(dt.date, ec);
    if (ec)
      return;
    set_time(dt.time, ec);
  }

  /// @brief Get the date.
  /// @return The date.
  Date get_date(std::error_code &ec) {
    logger_.info("getting date");
    uint8_t data[4];
    read_register(Registers::DATE, data, 4, ec);
    if (ec) {
      return {};
    }
    Date d;
    int base_year = (data[2] & CENTURY_BIT) ? 1900 : 2000;
    d.year = base_year + bcd2byte(data[3] & 0xff);
    d.month = bcd2byte(data[2] & 0x1f);
    d.weekday = bcd2byte(data[1] & 0x07);
    d.day = bcd2byte(data[0] & 0x3f);
    return d;
  }

  /// @brief Set the date.
  /// @param d The date.
  void set_date(const Date &d, std::error_code &ec) {
    logger_.info("setting date");
    uint8_t data[] = {byte2bcd(d.day), byte2bcd(d.weekday),
                      (uint8_t)(byte2bcd(d.month) | ((d.year < 2000) ? 0x80 : 0x00)),
                      byte2bcd(d.year % 100)};
    write_register(Registers::DATE, data, 4, ec);
  }

  /// @brief Get the time.
  /// @return The time.
  Time get_time(std::error_code &ec) {
    logger_.info("getting time");
    uint8_t data[3];
    read_register(Registers::TIME, data, 3, ec);
    if (ec) {
      return {};
    }
    Time t;
    t.hour = bcd2byte(data[2] & 0x3f);
    t.minute = bcd2byte(data[1] & 0x7f);
    t.second = bcd2byte(data[0] & 0x7f);
    return t;
  }

  /// @brief Set the time.
  /// @param t The time.
  void set_time(const Time &t, std::error_code &ec) {
    logger_.info("Setting time");
    uint8_t data[] = {byte2bcd(t.second), byte2bcd(t.minute), byte2bcd(t.hour)};
    write_register(Registers::TIME, data, 3, ec);
  }

protected:
  void init(std::error_code &ec) {
    logger_.info("initializing");
    uint8_t data[] = {0, 0};
    write_register(Registers::CONTROL_STATUS1, data, 2, ec);
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

  void read_register(Registers reg, uint8_t *data, size_t size, std::error_code &ec) {
    bool success = read_(DEFAULT_ADDRESS, (uint8_t)reg, data, size);
    if (!success) {
      ec = std::make_error_code(std::errc::io_error);
    }
  }

  void write_register(Registers reg, const uint8_t *data, size_t size, std::error_code &ec) {
    uint8_t buf[size + 1];
    buf[0] = (uint8_t)reg;
    memcpy(buf + 1, data, size);
    bool success = write_(DEFAULT_ADDRESS, buf, size + 1);
    if (!success) {
      ec = std::make_error_code(std::errc::io_error);
    }
  }

  write_fn write_;
  read_fn read_;
  espp::Logger logger_;
};
} // namespace espp

[[maybe_unused]] static bool operator==(const espp::Bm8563::Date &lhs,
                                        const espp::Bm8563::Date &rhs) {
  return lhs.year == rhs.year && lhs.month == rhs.month && lhs.weekday == rhs.weekday &&
         lhs.day == rhs.day;
}

[[maybe_unused]] static bool operator==(const espp::Bm8563::Time &lhs,
                                        const espp::Bm8563::Time &rhs) {
  return lhs.hour == rhs.hour && lhs.minute == rhs.minute && lhs.second == rhs.second;
}

[[maybe_unused]] static bool operator==(const espp::Bm8563::DateTime &lhs,
                                        const espp::Bm8563::DateTime &rhs) {
  return lhs.date == rhs.date && lhs.time == rhs.time;
}
// for allowing easy serialization/printing of the
// espp::Bm8563::Date, espp::Bm8563::Time, and espp::Bm8563::DateTime
template <> struct fmt::formatter<espp::Bm8563::Date> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext> auto format(espp::Bm8563::Date const &d, FormatContext &ctx) {
    return fmt::format_to(ctx.out(), "{{.year = {}, .month = {}, .weekday = {}, .day = {}}}",
                          d.year, d.month, d.weekday, d.day);
  }
};

template <> struct fmt::formatter<espp::Bm8563::Time> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext> auto format(espp::Bm8563::Time const &t, FormatContext &ctx) {
    return fmt::format_to(ctx.out(), "{{.hour = {}, .minute = {}, .second = {}}}", t.hour, t.minute,
                          t.second);
  }
};

template <> struct fmt::formatter<espp::Bm8563::DateTime> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::Bm8563::DateTime const &dt, FormatContext &ctx) {
    return fmt::format_to(ctx.out(), "{{.date = {}, .time = {}}}", dt.date, dt.time);
  }
};
