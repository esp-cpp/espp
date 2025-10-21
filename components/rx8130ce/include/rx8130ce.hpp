#pragma once

#include <chrono>
#include <ctime>
#include <functional>
#include <thread>

#include "base_peripheral.hpp"

namespace espp {
/// @brief The RX8130CE Real-Time Clock driver.
/// @details This driver is for the RX8130CE RTC chip from EPSON.
/// @tparam UseAddress Whether to use I2C addressing (default: true)
/// @note The RX8130CE is a temperature compensated crystal oscillator (TCXO) RTC
///       with I2C interface and various timing/alarm features.
/// @see https://support.epson.biz/td/api/doc_check.php?dl=brief_RX8130CE&lang=en
///
/// \section rx8130ce_ex1 Example
/// \snippet rx8130ce_example.cpp rx8130ce example
template <bool UseAddress = true>
class Rx8130ce : public espp::BasePeripheral<uint8_t, UseAddress> {
  using BasePeripheral<uint8_t, UseAddress>::write_fn;
  using BasePeripheral<uint8_t, UseAddress>::read_fn;
  using BasePeripheral<uint8_t, UseAddress>::set_address;
  using BasePeripheral<uint8_t, UseAddress>::set_write;
  using BasePeripheral<uint8_t, UseAddress>::set_read;
  using BasePeripheral<uint8_t, UseAddress>::write_u8_to_register;
  using BasePeripheral<uint8_t, UseAddress>::write_many_to_register;
  using BasePeripheral<uint8_t, UseAddress>::read_u8_from_register;
  using BasePeripheral<uint8_t, UseAddress>::read_many_from_register;
  using BasePeripheral<uint8_t, UseAddress>::clear_bits_in_register;
  using BasePeripheral<uint8_t, UseAddress>::set_bits_in_register;
  using BasePeripheral<uint8_t, UseAddress>::set_bits_in_register_by_mask;
  using BasePeripheral<uint8_t, UseAddress>::base_mutex_;
  using BasePeripheral<uint8_t, UseAddress>::logger_;

public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x32; ///< Default I2C address of the RX8130CE

  /// @brief Clock output frequencies available on the RX8130CE
  enum class ClockOutFrequency : uint8_t {
    FREQ_32768_HZ = 0x00, ///< 32.768 kHz
    FREQ_1024_HZ = 0x01,  ///< 1.024 kHz
    FREQ_1_HZ = 0x02,     ///< 1 Hz
    DISABLED = 0x03,      ///< Clock output disabled
  };

  /// @brief Timer clock source frequencies
  enum class TimerClockSource : uint8_t {
    FREQ_4096_HZ = 0x00,   ///< 4.096 kHz
    FREQ_64_HZ = 0x01,     ///< 64 Hz
    FREQ_1_HZ = 0x02,      ///< 1 Hz
    FREQ_1_60_HZ = 0x03,   ///< 1/60 Hz (1 minute)
    FREQ_1_3600_HZ = 0x04, ///< 1/3600 Hz (1 hour)
  };

  /// @brief The date structure (device-specific format)
  struct Date {
    uint16_t year;   ///< The year (full 4-digit year)
    uint8_t month;   ///< The month (1-12)
    uint8_t weekday; ///< The day of the week (0-6, Sunday=0)
    uint8_t day;     ///< The day of the month (1-31)

    bool operator==(const Date &other) const {
      return year == other.year && month == other.month && weekday == other.weekday &&
             day == other.day;
    }

    bool operator!=(const Date &other) const { return !(*this == other); }
  };

  /// @brief The time structure (device-specific format)
  struct Time {
    uint8_t hour;   ///< The hour (0-23)
    uint8_t minute; ///< The minute (0-59)
    uint8_t second; ///< The second (0-59)

    bool operator==(const Time &other) const {
      return hour == other.hour && minute == other.minute && second == other.second;
    }

    bool operator!=(const Time &other) const { return !(*this == other); }
  };

  /// @brief The date and time structure (device-specific format)
  struct DateTime {
    Date date; ///< The date
    Time time; ///< The time

    bool operator==(const DateTime &other) const {
      return date == other.date && time == other.time;
    }

    bool operator!=(const DateTime &other) const { return !(*this == other); }
  };

  /// @brief Configuration structure for the Rx8130ce
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS;            ///< I2C address of the device
    BasePeripheral<uint8_t, UseAddress>::write_fn write; ///< Write function for the I2C bus
    BasePeripheral<uint8_t, UseAddress>::read_fn read;   ///< Read function for the I2C bus
    bool auto_init{true}; ///< Whether to automatically initialize the device
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log level for the device
  };

  /// @brief Construct a new Rx8130ce object
  /// @param config The configuration for the device
  explicit Rx8130ce(const Config &config)
      : BasePeripheral<uint8_t, UseAddress>({}, "Rx8130ce", config.log_level) {
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

  /// @brief Initialize the device
  /// @param ec The error code to set if an error occurs
  /// @return True if the device was initialized successfully, false otherwise
  bool init(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    // software reset
    if (!reset(ec)) {
      return false;
    }

    logger_.debug("RX8130CE initialized successfully");
    return true;
  }

  /// @brief Reset the RX8130CE device
  /// @param ec The error code to set if an error occurs
  /// @return True if the device was reset successfully, false otherwise
  bool reset(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    // From figure 43, pp. 57 of the RX8130CE datasheet:
    // write 0x00 to 0x1E
    // write 0x80 to 0x1E
    // write 0x6C to 0x50
    // write 0x01 to 0x53
    // write 0x03 to 0x66
    // write 0x02 to 0x6B
    // write 0x01 to 0x6B

    write_u8_to_register(0x1E, 0x00, ec);
    if (ec)
      return false;
    write_u8_to_register(0x1E, 0x80, ec);
    if (ec)
      return false;
    write_u8_to_register(0x50, 0x6C, ec);
    if (ec)
      return false;
    write_u8_to_register(0x53, 0x01, ec);
    if (ec)
      return false;
    write_u8_to_register(0x66, 0x03, ec);
    if (ec)
      return false;
    write_u8_to_register(0x6B, 0x02, ec);
    if (ec)
      return false;
    write_u8_to_register(0x6B, 0x01, ec);
    if (ec)
      return false;

    // wait 125 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(125));

    return !ec;
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

    uint8_t data[7];
    data[0] = decimal_to_bcd(time.tm_sec);
    data[1] = decimal_to_bcd(time.tm_min);
    data[2] = decimal_to_bcd(time.tm_hour);
    data[3] = decimal_to_bcd(time.tm_wday);
    data[4] = decimal_to_bcd(time.tm_mday);
    data[5] = decimal_to_bcd(time.tm_mon + 1);
    data[6] = decimal_to_bcd((time.tm_year + 1900) % 100);

    write_many_to_register(static_cast<uint8_t>(Register::SEC), data, 7, ec);
    if (ec)
      return false;

    logger_.debug("Time set: {:%Y-%m-%d %H:%M:%S}", time);

    return true;
  }

  /// @brief Get the time using std::tm structure
  /// @param ec The error code to set if an error occurs
  /// @return The time from the device
  std::tm get_time(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    uint8_t data[7];
    read_many_from_register(static_cast<uint8_t>(Register::SEC), data, 7, ec);
    if (ec) {
      return {};
    }

    std::tm time = {};
    time.tm_sec = bcd_to_decimal(data[0] & 0x7F);
    time.tm_min = bcd_to_decimal(data[1] & 0x7F);
    time.tm_hour = bcd_to_decimal(data[2] & 0x3F);
    time.tm_wday = bcd_to_decimal(data[3] & 0x07);
    time.tm_mday = bcd_to_decimal(data[4] & 0x3F);
    time.tm_mon = bcd_to_decimal(data[5] & 0x1F) - 1;
    time.tm_year =
        bcd_to_decimal(data[6]) + 100; // tm_year is years since 1900, assuming 21st century

    // Calculate tm_yday (day of year) if needed
    time.tm_yday = -1;  // Let mktime calculate it
    time.tm_isdst = -1; // Let system determine DST

    // Normalize the time structure
    mktime(&time);

    logger_.debug("Time read: {:%Y-%m-%d %H:%M:%S}", time);

    return time;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Device-specific APIs (for advanced features)
  /////////////////////////////////////////////////////////////////////////////

  /// @brief Get the date and time using device-specific structures
  /// @param ec The error code to set if an error occurs
  /// @return The date and time
  DateTime get_date_time(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    // Convert std::tm to device-specific format
    auto tm_time = get_time(ec);
    if (ec)
      return {};

    DateTime dt;
    dt.date.year = tm_time.tm_year + 1900;
    dt.date.month = tm_time.tm_mon + 1;
    dt.date.weekday = tm_time.tm_wday;
    dt.date.day = tm_time.tm_mday;
    dt.time.hour = tm_time.tm_hour;
    dt.time.minute = tm_time.tm_min;
    dt.time.second = tm_time.tm_sec;

    return dt;
  }

  /// @brief Set the date and time using device-specific structures
  /// @param dt The date and time to set
  /// @param ec The error code to set if an error occurs
  void set_date_time(const DateTime &dt, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    // Convert to std::tm and use the std::tm API
    std::tm time = {};
    time.tm_year = dt.date.year - 1900;
    time.tm_mon = dt.date.month - 1;
    time.tm_mday = dt.date.day;
    time.tm_wday = dt.date.weekday;
    time.tm_hour = dt.time.hour;
    time.tm_min = dt.time.minute;
    time.tm_sec = dt.time.second;

    set_time(time, ec);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Error Detection Functions
  /////////////////////////////////////////////////////////////////////////////

  /// @brief Check if the voltage loss flag is set
  /// @param ec The error code to set if an error occurs
  /// @return True if a voltage loss has occurred, false otherwise
  bool get_voltage_loss_flag(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t flag = read_u8_from_register(static_cast<uint8_t>(Register::FLAG), ec);
    return !ec && (flag & VOLTAGE_LOSS_FLAG_BIT);
  }

  /// @brief Clear the voltage loss flag
  /// @param ec The error code to set if an error occurs
  /// @return True if the flag was cleared successfully, false otherwise
  bool clear_voltage_loss_flag(std::error_code &ec) {
    set_bits_in_register_by_mask(static_cast<uint8_t>(Register::FLAG), VOLTAGE_LOSS_FLAG_BIT, 0x00,
                                 ec);
    return !ec;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Alarm Functions
  /////////////////////////////////////////////////////////////////////////////

  /// @brief Set an alarm using std::tm structure
  /// @param alarm_time The time to trigger the alarm
  /// @param is_week_day_target True to use week day as target of alarm
  ///        function, false to use month day
  /// @param ec The error code to set if an error occurs
  /// @return True if the alarm was set successfully, false otherwise
  /// @note If is_week_day_target is true, the alarm will trigger on the specified
  ///       day of the month. If false, it will trigger on the specified day of
  ///       the week.
  /// @note This will match exactly the minute, hour, and day/week specified
  ///       because it will set all the alarm bits to 0 (enabled for exact
  ///       match).
  bool set_alarm(const std::tm &alarm_time, bool is_week_day_target, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    // Note: this will match exactly the minute,hour and day/week specified
    //       because it will set the alarm registers directly, clearing the AE
    //       (0x80) bits in each register.

    // write the alarm registers
    uint8_t data[3];
    data[0] = decimal_to_bcd(alarm_time.tm_min);
    data[1] = decimal_to_bcd(alarm_time.tm_hour);
    if (is_week_day_target) {
      data[2] = decimal_to_bcd(alarm_time.tm_wday);
    } else {
      data[2] = decimal_to_bcd(alarm_time.tm_mday);
    }
    write_many_to_register(static_cast<uint8_t>(Register::ALARM_MIN), data, 3, ec);
    if (ec)
      return false;

    // Set week/day alarm bit
    set_bits_in_register_by_mask(static_cast<uint8_t>(Register::EXTENSION), WEEK_DAY_ALARM_BIT,
                                 is_week_day_target ? WEEK_DAY_ALARM_BIT : 0x00, ec);

    return !ec;
  }

  /// @brief Enable or disable the alarm interrupt
  /// @param enabled True to enable, false to disable
  /// @param ec The error code to set if an error occurs
  /// @return True if the operation was successful, false otherwise
  bool set_alarm_interrupt_enabled(bool enabled, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t value = enabled ? ALARM_INT_ENABLE_BIT : 0x00;
    set_bits_in_register_by_mask(static_cast<uint8_t>(Register::CONTROL), ALARM_INT_ENABLE_BIT,
                                 value, ec);
    return !ec;
  }

  /// @brief Disable the alarm
  /// @param ec The error code to set if an error occurs
  /// @return True if the alarm was disabled successfully, false otherwise
  bool disable_alarm(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    uint8_t ctrl = read_u8_from_register(static_cast<uint8_t>(Register::CONTROL), ec);
    if (ec)
      return false;

    ctrl &= ~ALARM_INT_ENABLE_BIT; // Disable alarm interrupt
    write_u8_to_register(static_cast<uint8_t>(Register::CONTROL), ctrl, ec);

    return !ec;
  }

  /// @brief Check if the alarm flag is set
  /// @param ec The error code to set if an error occurs
  /// @return True if the alarm is triggered, false otherwise
  bool is_alarm_triggered(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    uint8_t flag = read_u8_from_register(static_cast<uint8_t>(Register::FLAG), ec);
    return !ec && (flag & ALARM_FLAG_BIT);
  }

  /// @brief Clear the alarm flag
  /// @param ec The error code to set if an error occurs
  /// @return True if the flag was cleared successfully, false otherwise
  bool clear_alarm_flag(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    uint8_t flag = read_u8_from_register(static_cast<uint8_t>(Register::FLAG), ec);
    if (ec)
      return false;

    flag &= ~ALARM_FLAG_BIT; // Clear alarm flag
    write_u8_to_register(static_cast<uint8_t>(Register::FLAG), flag, ec);

    return !ec;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Timer Functions
  /////////////////////////////////////////////////////////////////////////////

  /// @brief Set the countdown timer
  /// @param value Timer value (0-4095)
  /// @param clock_source Timer clock source
  /// @param ec The error code to set if an error occurs
  /// @return True if the timer was set successfully, false otherwise
  bool set_timer(uint16_t value, TimerClockSource clock_source, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    // ensure timer enable (TE) bit is 0 before writing to timer registers
    set_bits_in_register_by_mask(static_cast<uint8_t>(Register::EXTENSION), TIMER_ENABLE_BIT, 0x00,
                                 ec);

    // Set timer value (16-bit)
    uint8_t timer_low = value & 0xFF;
    uint8_t timer_high = (value >> 8);

    write_u8_to_register(static_cast<uint8_t>(Register::TIMER_COUNTER0), timer_low, ec);
    if (ec)
      return false;
    write_u8_to_register(static_cast<uint8_t>(Register::TIMER_COUNTER1), timer_high, ec);
    if (ec)
      return false;

    // Configure timer control
    uint8_t timer_select =
        static_cast<uint8_t>(clock_source); // TSEL[0:2] bits in Extension register as bits 0-2
    set_bits_in_register_by_mask(static_cast<uint8_t>(Register::EXTENSION), 0x07, timer_select, ec);
    if (ec)
      return false;

    // now start the timer
    set_bits_in_register_by_mask(static_cast<uint8_t>(Register::EXTENSION), TIMER_ENABLE_BIT,
                                 TIMER_ENABLE_BIT, ec);

    return !ec;
  }

  /// @brief Stop the timer
  /// @param ec The error code to set if an error occurs
  /// @return True if the timer was stopped successfully, false otherwise
  bool stop_timer(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    // read extension register
    uint8_t ext = read_u8_from_register(static_cast<uint8_t>(Register::EXTENSION), ec);
    if (ec)
      return false;

    ext &= ~TIMER_ENABLE_BIT; // Clear timer enable bit

    write_u8_to_register(static_cast<uint8_t>(Register::EXTENSION), ext, ec);
    return !ec;
  }

  /// @brief Enable or disable the timer interrupt
  /// @param enabled True to enable, false to disable
  /// @param ec The error code to set if an error occurs
  /// @return True if the operation was successful, false otherwise
  bool set_timer_interrupt_enabled(bool enabled, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t value = enabled ? TIMER_INT_ENABLE_BIT : 0x00;
    set_bits_in_register_by_mask(static_cast<uint8_t>(Register::CONTROL), TIMER_INT_ENABLE_BIT,
                                 value, ec);
    return !ec;
  }

  /// @brief Check if the timer flag is set
  /// @param ec The error code to set if an error occurs
  /// @return True if the timer has expired, false otherwise
  bool is_timer_expired(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    uint8_t flag = read_u8_from_register(static_cast<uint8_t>(Register::FLAG), ec);
    return !ec && (flag & TIMER_FLAG_BIT);
  }

  /// @brief Clear the timer flag
  /// @param ec The error code to set if an error occurs
  /// @return True if the flag was cleared successfully, false otherwise
  bool clear_timer_flag(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    uint8_t flag = read_u8_from_register(static_cast<uint8_t>(Register::FLAG), ec);
    if (ec)
      return false;

    flag &= ~TIMER_FLAG_BIT; // Clear timer flag
    write_u8_to_register(static_cast<uint8_t>(Register::FLAG), flag, ec);

    return !ec;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Clock Output Functions
  /////////////////////////////////////////////////////////////////////////////

  /// @brief Configure the clock output
  /// @param frequency The output frequency
  /// @param ec The error code to set if an error occurs
  /// @return True if the clock output was configured successfully, false otherwise
  bool set_clock_output(ClockOutFrequency frequency, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    uint8_t ctrl = read_u8_from_register(static_cast<uint8_t>(Register::EXTENSION), ec);
    if (ec)
      return false;

    ctrl &= ~(0x03 << FSEL0_BIT_POS);                         // Clear FSEL bits
    ctrl |= static_cast<uint8_t>(frequency) << FSEL0_BIT_POS; // Set new frequency bits

    write_u8_to_register(static_cast<uint8_t>(Register::EXTENSION), ctrl, ec);

    return !ec;
  }

protected:
  // voltage loss flag bit
  static constexpr uint8_t VOLTAGE_LOSS_FLAG_BIT =
      0x02; ///< Voltage loss flag bit in FLAG register (0x1D)

  // timer interrupt bits
  static constexpr uint8_t TIMER_INT_ENABLE_BIT =
      0x10; ///< Timer interrupt enable bit in CONTROL register 0 (0x1E)
  static constexpr uint8_t TIMER_FLAG_BIT = 0x10; ///< Timer flag bit in FLAG register (0x1D)
  static constexpr uint8_t TIMER_ENABLE_BIT =
      0x10; ///< Timer enable bit in EXTENSION register (0x1C)

  // Time update interrupt function
  static constexpr uint8_t TIME_UPDATE_INT_ENABLE_BIT =
      0x20; ///< Time update interrupt enable bit in Extension (0x1C) register
  static constexpr uint8_t TIME_UPDATE_FLAG_BIT =
      0x20; ///< Time update flag in FLAG register (0x1D)
  static constexpr uint8_t TIME_UPDATE_INT_SELECT_BIT =
      0x20; ///< Update select bit in CONTROL register (0x1E).  0 (default) selects 1Hz, 1 selects 1
            ///< minute

  // alarm interupt bits
  static constexpr uint8_t ALARM_MINUTE_ENABLE_BIT =
      0x80; ///< Alarm minute enable bit in Minute Alarm register (0x17)
  static constexpr uint8_t ALARM_HOUR_ENABLE_BIT =
      0x80; ///< Alarm hour enable bit in Hour Alarm register (0x18)
  static constexpr uint8_t ALARM_WEEK_DAY_ENABLE_BIT =
      0x80; ///< Alarm day enable bit in Week/Day Alarm register (0x19)
  static constexpr uint8_t ALARM_INT_ENABLE_BIT =
      0x08; ///< Alarm interrupt enable bit in CONTROL register 0 (0x1E)
  static constexpr uint8_t ALARM_FLAG_BIT = 0x08; ///< Alarm flag bit in FLAG register (0x1D)
  static constexpr uint8_t WEEK_DAY_ALARM_BIT =
      0x08; ///< Weekday alarm bit in Extension register (0x1C)

  // clock output bits
  static constexpr uint8_t FSEL0_BIT_POS = 6; // FSEL0 is bit 6
  static constexpr uint8_t FSEL1_BIT_POS = 7; // FSEL1 is bit 7

  /// @brief Register addresses for the RX8130CE
  enum class Register : uint8_t {
    SEC = 0x10,             ///< Seconds register
    MIN = 0x11,             ///< Minutes register
    HOUR = 0x12,            ///< Hours register
    WDAY = 0x13,            ///< Weekday register
    MDAY = 0x14,            ///< Day register
    MONTH = 0x15,           ///< Month register
    YEAR = 0x16,            ///< Year register
    ALARM_MIN = 0x17,       ///< Alarm minutes register
    ALARM_HOUR = 0x18,      ///< Alarm hours register
    ALARM_WDAY_MDAY = 0x19, ///< Alarm weekday/day register
    TIMER_COUNTER0 = 0x1A,  ///< Timer counter 0 register
    TIMER_COUNTER1 = 0x1B,  ///< Timer counter 1 register
    EXTENSION = 0x1C,       ///< Extension register
    FLAG = 0x1D,            ///< Flag register
    CONTROL = 0x1E,         ///< Control register
    TIMER_CONTROL = 0x1F,   ///< Timer control register

    USER_RAM_START = 0x20, ///< Start of user RAM
    USER_RAM_END = 0x23,   ///< End of user RAM
  };

  /// @brief Convert decimal to BCD
  /// @param decimal The decimal value
  /// @return The BCD value
  static uint8_t decimal_to_bcd(uint8_t decimal) { return ((decimal / 10) << 4) | (decimal % 10); }

  /// @brief Convert BCD to decimal
  /// @param bcd The BCD value
  /// @return The decimal value
  static uint8_t bcd_to_decimal(uint8_t bcd) { return ((bcd >> 4) * 10) + (bcd & 0x0F); }
};

} // namespace espp
