#include <chrono>
#include <thread>

#include "m5stack-tab5.hpp"

namespace espp {

//////////////////////////////////////////////////////////////////////////
// Battery Monitoring
//////////////////////////////////////////////////////////////////////////

bool M5StackTab5::initialize_battery_monitoring() {
  logger_.info("Initializing battery monitoring (INA226)");

  std::error_code ec;
  battery_monitor_i2c_device_ = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = 0x41,
          .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!battery_monitor_i2c_device_) {
    logger_.error("Could not initialize battery monitor I2C device: {}", ec.message());
    return false;
  }

  // INA226 connected to the internal I2C bus; setup with typical values.
  BatteryMonitor::Config cfg{
      .device_address = 0x41, // NOTE: not the default address, the ES7210 uses 0x40
      .averaging = BatteryMonitor::Avg::AVG_16,
      .bus_conv_time = BatteryMonitor::ConvTime::MS_1_1,
      .shunt_conv_time = BatteryMonitor::ConvTime::MS_1_1,
      .mode = BatteryMonitor::Mode::SHUNT_BUS_CONT,
      .current_lsb = 0.0005f,          // 0.5 mA / LSB
      .shunt_resistance_ohms = 0.005f, // 5 mΩ (adjust if different on board)
      .probe = espp::make_i2c_addressed_probe(battery_monitor_i2c_device_),
      .write = espp::make_i2c_addressed_write(battery_monitor_i2c_device_),
      .read_register = espp::make_i2c_addressed_read_register(battery_monitor_i2c_device_),
      .write_then_read = espp::make_i2c_addressed_write_then_read(battery_monitor_i2c_device_),
      .auto_init = true,
      .log_level = espp::Logger::Verbosity::WARN,
  };

  battery_monitor_ = std::make_shared<BatteryMonitor>(cfg);

  if (!battery_monitor_->initialize(ec) || ec) {
    logger_.error("Battery monitor (INA226) initialization failed: {}", ec.message());
    battery_monitor_.reset();
    return false;
  }

  // The INA226's data registers read as zeros until its first averaged
  // conversion completes (averaging x (bus + shunt conversion time): 35 ms with
  // the settings above), and a 0 V reading looks exactly like an empty battery
  // bay to read_battery_status(). Wait for the first conversion so the first
  // read after this returns is real. The timeout is a generous multiple of the
  // expected time so a slow I2C bus or a busy scheduler does not trip it.
  {
    static constexpr uint32_t conversion_poll_ms = 5;
    static constexpr uint32_t conversion_timeout_factor = 4;
    const uint32_t expected_ms =
        BatteryMonitor::conversion_time_us(cfg.averaging, cfg.bus_conv_time, cfg.shunt_conv_time) /
        1000;
    const uint32_t timeout_ms =
        std::max<uint32_t>(conversion_timeout_factor * expected_ms, 10 * conversion_poll_ms);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    bool ready = false;
    while (!ready && std::chrono::steady_clock::now() < deadline) {
      ready = battery_monitor_->conversion_ready(ec);
      if (ec) {
        break;
      }
      if (!ready) {
        std::this_thread::sleep_for(std::chrono::milliseconds(conversion_poll_ms));
      }
    }
    if (!ready) {
      logger_.warn("Battery monitor (INA226) first conversion not ready after {} ms (expected "
                   "{} ms){}{}",
                   timeout_ms, expected_ms, ec ? ": " : "", ec ? ec.message() : "");
    }
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
  // The INA226 sits on the pack line. With no NP-F550 in the bay it reads the
  // charger's open-circuit output (about 2 V, measured); a real 2S pack never
  // reads below its protection cut-off (about 5 V), so anything under this is
  // an empty bay, not a flat battery.
  static constexpr float pack_present_min_volts = 4.0f;

  std::error_code ec;
  std::lock_guard<std::mutex> lock(battery_mutex_);
  float vbus = battery_monitor_->bus_voltage_volts(ec);
  [[maybe_unused]] float vshunt = battery_monitor_->shunt_voltage_volts(ec);
  float current_a = battery_monitor_->current_amps(ec);
  float power_w = battery_monitor_->power_watts(ec);
  if (!ec) {
    logger_.debug("Battery status updated");
    battery_status_.voltage_v = vbus;
    battery_status_.current_ma = current_a * 1000.0f;
    battery_status_.power_mw = power_w * 1000.0f;
    battery_status_.is_present = vbus >= pack_present_min_volts;
    if (battery_status_.is_present) {
      // Basic SoC estimate from voltage (very rough placeholder)
      float v = vbus;
      float soc = (v - pack_voltage_min) / (pack_voltage_max - pack_voltage_min);
      soc = std::clamp(soc, 0.0f, 1.0f);
      battery_status_.charge_percent = soc * 100.0f;
      // only charging if the bit says we are and the current is < -1.0 mA
      battery_status_.is_charging = get_charging_status() && battery_status_.current_ma < -1.0f;
    } else {
      battery_status_.charge_percent = 0.0f;
      battery_status_.is_charging = false;
    }
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

void M5StackTab5::set_charging_pause_on_playback_enabled(bool enable) {
  charging_pause_on_playback_enabled_.store(enable, std::memory_order_relaxed);
  if (!enable) {
    resume_charging_after_audio();
  }
}

void M5StackTab5::update_charging_for_audio(bool audio_active) {
  // Runs on the audio playback task once per block (about 60 Hz). The IO
  // expander write happens only on the two transitions; the common case is a
  // clock read and a compare.
  if (!charging_pause_on_playback_enabled_.load(std::memory_order_relaxed)) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (audio_active) {
    audio_last_active_ = now;
    std::lock_guard<std::mutex> lock(charging_mutex_);
    if (!charging_paused_for_audio_) {
      charging_paused_for_audio_ = true;
      if (charging_requested_) {
        set_io_expander_output(0x44, IO44_BIT_CHG_EN, false);
      }
    }
    return;
  }
  if (now - audio_last_active_ >= std::chrono::milliseconds(CHARGING_RESUME_DELAY_MS)) {
    resume_charging_after_audio();
  }
}

void M5StackTab5::resume_charging_after_audio() {
  std::lock_guard<std::mutex> lock(charging_mutex_);
  if (charging_paused_for_audio_) {
    charging_paused_for_audio_ = false;
    if (charging_requested_) {
      set_io_expander_output(0x44, IO44_BIT_CHG_EN, true);
    }
  }
}

void M5StackTab5::enable_battery_charging(bool enable) {
  // Control charging via IP2326 charge management IC This is done through the
  // PI4IOE5V6408 GPIO expander
  logger_.info("Battery charging {}", enable ? "enabled" : "disabled");
  set_charging_enabled(enable);
}

} // namespace espp
