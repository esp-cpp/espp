#include "m5stack-tab5.hpp"

namespace espp {

//////////////////////////////////////////////////////////////////////////
// Battery Monitoring
//////////////////////////////////////////////////////////////////////////

bool M5StackTab5::initialize_battery_monitoring() {
  logger_.info("Initializing battery monitoring (INA226)");

  // INA226 connected to the internal I2C bus; setup with typical values.
  BatteryMonitor::Config cfg{
      .device_address = 0x41, // NOTE: not the default address, the ES7210 uses 0x40
      .averaging = BatteryMonitor::Avg::AVG_16,
      .bus_conv_time = BatteryMonitor::ConvTime::MS_1_1,
      .shunt_conv_time = BatteryMonitor::ConvTime::MS_1_1,
      .mode = BatteryMonitor::Mode::SHUNT_BUS_CONT,
      .current_lsb = 0.0005f,          // 0.5 mA / LSB
      .shunt_resistance_ohms = 0.005f, // 5 mΩ (adjust if different on board)
      .probe = std::bind(&espp::I2c::probe_device, &internal_i2c(), std::placeholders::_1),
      .write = std::bind(&espp::I2c::write, &internal_i2c(), std::placeholders::_1,
                         std::placeholders::_2, std::placeholders::_3),
      .read_register =
          std::bind(&espp::I2c::read_at_register, &internal_i2c(), std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
      .write_then_read = std::bind(&espp::I2c::write_read, &internal_i2c(), std::placeholders::_1,
                                   std::placeholders::_2, std::placeholders::_3,
                                   std::placeholders::_4, std::placeholders::_5),
      .auto_init = true,
      .log_level = espp::Logger::Verbosity::WARN,
  };

  battery_monitor_ = std::make_shared<BatteryMonitor>(cfg);

  std::error_code ec;
  if (!battery_monitor_->initialize(ec) || ec) {
    logger_.error("Battery monitor (INA226) initialization failed: {}", ec.message());
    battery_monitor_.reset();
    return false;
  }

  battery_monitoring_initialized_ = true;

  // Initialize battery status with default values
  {
    std::lock_guard<std::mutex> lock(battery_mutex_);
    battery_status_ = {.voltage_v = 0.0f,
                       .current_ma = 0.0f,
                       .power_mw = 0.0f,
                       .charge_percent = 0.0f,
                       .is_charging = false,
                       .is_present = false};
  }

  logger_.info("Battery monitoring initialized");
  return true;
}

M5StackTab5::BatteryStatus M5StackTab5::get_battery_status() const { return battery_status_; }

bool M5StackTab5::update_battery_status() {
  if (!battery_monitoring_initialized_) {
    return false;
  }

  if (!battery_monitor_) {
    return false;
  }

  static constexpr float cell_voltage_min = 3.2f; // volts
  static constexpr float cell_voltage_max = 4.2f; // volts
  // Cells are in a 2S1P configuration
  static constexpr float num_cells = 2.0f;
  static constexpr float pack_voltage_min = cell_voltage_min * num_cells;
  static constexpr float pack_voltage_max = cell_voltage_max * num_cells;

  std::error_code ec;
  std::lock_guard<std::mutex> lock(battery_mutex_);
  float vbus = battery_monitor_->bus_voltage_volts(ec);
  float vshunt = battery_monitor_->shunt_voltage_volts(ec);
  float current_a = battery_monitor_->current_amps(ec);
  float power_w = battery_monitor_->power_watts(ec);
  if (!ec) {
    logger_.debug("Battery status updated");
    battery_status_.voltage_v = vbus;
    battery_status_.current_ma = current_a * 1000.0f;
    battery_status_.power_mw = power_w * 1000.0f;
    // Basic SoC estimate from voltage (very rough placeholder)
    float v = vbus;
    float soc = (v - pack_voltage_min) / (pack_voltage_max - pack_voltage_min);
    soc = std::clamp(soc, 0.0f, 1.0f);
    battery_status_.charge_percent = soc * 100.0f;
    // only charging if the bit sayus we are and  the current is < -1.0 mA
    battery_status_.is_charging = get_charging_status() && battery_status_.current_ma < -1.0f;
    battery_status_.is_present = true;
  } else {
    logger_.warn("Battery status not updated: {}", ec.message());
    battery_status_.voltage_v = 0.0f;
    battery_status_.current_ma = 0.0f;
    battery_status_.power_mw = 0.0f;
    battery_status_.charge_percent = 0.0f;
    battery_status_.is_charging = false;
    battery_status_.is_present = false;
  }

  return !ec;
}

M5StackTab5::BatteryStatus M5StackTab5::read_battery_status() {
  if (!update_battery_status()) {
    return {};
  }
  return get_battery_status();
}

void M5StackTab5::enable_battery_charging(bool enable) {
  // Control charging via IP2326 charge management IC This is done through the
  // PI4IOE5V6408 GPIO expander
  logger_.info("Battery charging {}", enable ? "enabled" : "disabled");
  set_charging_enabled(enable);
}

//////////////////////////////////////////////////////////////////////////
// Real-Time Clock
//////////////////////////////////////////////////////////////////////////

bool M5StackTab5::initialize_rtc() {
  logger_.info("Initializing RX8130CE real-time clock");

  // TODO: Implement RX8130CE RTC initialization
  // This would involve:
  // 1. I2C communication with RX8130CE
  // 2. Clock configuration and calibration
  // 3. Alarm and interrupt setup
  // 4. Battery backup configuration

  rtc_initialized_ = true;
  logger_.info("RTC initialization placeholder completed");
  return true;
}

bool M5StackTab5::set_rtc_time(uint64_t unix_timestamp) {
  if (!rtc_initialized_) {
    logger_.error("RTC not initialized");
    return false;
  }

  // TODO: Convert unix timestamp to RTC format and write to RX8130CE
  logger_.info("RTC time set to {}", unix_timestamp);
  return true;
}

uint64_t M5StackTab5::get_rtc_time() {
  if (!rtc_initialized_) {
    logger_.warn("RTC not initialized");
    return 0;
  }

  // TODO: Read time from RX8130CE and convert to unix timestamp
  // For now, return system time
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return tv.tv_sec;
}

bool M5StackTab5::set_rtc_wakeup(uint32_t seconds_from_now, std::function<void()> callback) {
  if (!rtc_initialized_) {
    logger_.error("RTC not initialized");
    return false;
  }

  rtc_wakeup_callback_ = callback;

  // TODO: Configure RX8130CE alarm for wake-up
  // This would involve:
  // 1. Calculate alarm time
  // 2. Configure RTC alarm registers
  // 3. Enable alarm interrupt
  // 4. Setup ESP32-P4 wake-up source

  logger_.info("RTC wake-up set for {} seconds from now", seconds_from_now);
  return true;
}

} // namespace espp
