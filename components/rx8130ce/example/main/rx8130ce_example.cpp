#include <chrono>
#include <vector>

#include "i2c.hpp"
#include "rx8130ce.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "RX8130CE Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting");

  //! [rx8130ce example]
  // make the I2C that we'll use to communicate
  static constexpr auto i2c_port = I2C_NUM_0;
  static constexpr auto i2c_clock_speed = CONFIG_EXAMPLE_I2C_CLOCK_SPEED_HZ;
  static constexpr gpio_num_t i2c_sda = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO;
  static constexpr gpio_num_t i2c_scl = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO;
  espp::I2c i2c({.port = i2c_port,
                 .sda_io_num = i2c_sda,
                 .scl_io_num = i2c_scl,
                 .sda_pullup_en = GPIO_PULLUP_ENABLE,
                 .scl_pullup_en = GPIO_PULLUP_ENABLE,
                 .clk_speed = i2c_clock_speed});

  using Rtc = espp::Rx8130ce<>;

  // Configure the RX8130CE
  Rtc::Config config{.device_address = Rtc::DEFAULT_ADDRESS,
                     .write = std::bind_front(&espp::I2c::write, &i2c),
                     .read = std::bind_front(&espp::I2c::read, &i2c),
                     .log_level = espp::Logger::Verbosity::INFO};

  // Create RTC instance
  Rtc rtc(config);

  // Test std::tm based API
  logger.info("Testing std::tm based API");

  // Set time using std::tm
  std::tm time = {};
  time.tm_year = 124; // 2024 - 1900
  time.tm_mon = 0;    // January (0-based)
  time.tm_mday = 15;  // 15th
  time.tm_hour = 14;  // 2 PM
  time.tm_min = 30;
  time.tm_sec = 45;
  time.tm_wday = 1; // Monday

  std::error_code ec;
  if (rtc.set_time(time, ec)) {
    logger.info("Time set successfully");
  } else {
    logger.error("Failed to set time: {}", ec.message());
  }

  // Get time using std::tm
  auto current_time = rtc.get_time(ec);
  if (!ec) {
    logger.info("Current time: {}-{:02d}-{:02d} {:02d}:{:02d}:{:02d} (wday: {})",
                current_time.tm_year + 1900, current_time.tm_mon + 1, current_time.tm_mday,
                current_time.tm_hour, current_time.tm_min, current_time.tm_sec,
                current_time.tm_wday);
  } else {
    logger.error("Failed to get time: {}", ec.message());
  }

  // Test device-specific API
  logger.info("Testing device-specific API");

  auto dt = rtc.get_date_time(ec);
  if (!ec) {
    logger.info("Device format - Date: {}-{:02d}-{:02d} (wday: {}), Time: {:02d}:{:02d}:{:02d}",
                dt.date.year, dt.date.month, dt.date.day, dt.date.weekday, dt.time.hour,
                dt.time.minute, dt.time.second);
  }

  // Test alarm functionality
  logger.info("Testing alarm functionality");

  std::tm alarm_time = current_time;
  alarm_time.tm_sec += 10; // Alarm in 10 seconds
  mktime(&alarm_time);     // Normalize the time

  if (rtc.set_alarm(alarm_time, false, ec)) {
    logger.info("Alarm set for {:02d}:{:02d}:{:02d}", alarm_time.tm_hour, alarm_time.tm_min,
                alarm_time.tm_sec);
  }

  // Test timer functionality
  logger.info("Testing timer functionality");

  // Set a 5-second timer
  if (rtc.set_timer(5, Rtc::TimerClockSource::FREQ_1_HZ, ec)) {
    logger.info("Timer set for 5 seconds");
  }

  // Configure clock output
  logger.info("Configuring clock output to 1Hz");
  rtc.set_clock_output(Rtc::ClockOutFrequency::FREQ_1_HZ, ec);

  // Main loop - monitor alarms and timer
  logger.info("Monitoring RTC events...");

  while (true) {
    std::this_thread::sleep_for(1s);

    // Get current time
    current_time = rtc.get_time(ec);
    if (!ec) {
      logger.info("Time: {:02d}:{:02d}:{:02d}", current_time.tm_hour, current_time.tm_min,
                  current_time.tm_sec);
    }

    // Check alarm
    if (rtc.is_alarm_triggered(ec) && !ec) {
      logger.info("🚨 ALARM TRIGGERED!");
      rtc.clear_alarm_flag(ec);
    }

    // Check timer
    if (rtc.is_timer_expired(ec) && !ec) {
      logger.info("⏰ TIMER EXPIRED!");
      rtc.clear_timer_flag(ec);

      // Restart timer for another 5 seconds
      rtc.set_timer(5, Rtc::TimerClockSource::FREQ_1_HZ, ec);
      logger.info("Timer restarted");
    }
  }
  //! [rx8130ce example]
}
