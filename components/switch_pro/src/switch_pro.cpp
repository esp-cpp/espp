#include "switch_pro.hpp"

#include <algorithm>

using namespace espp;

SwitchPro::SwitchPro(const Config &config)
    : BaseComponent("SwitchPro", config.log_level) {
  // start the input-report counter timer
  counter_timer_.periodic(counter_period_us);

  // copy the SPI ROM calibration / config blobs
  std::copy(std::begin(sp::spi_rom_data_60), std::end(sp::spi_rom_data_60),
            spi_rom_factory_data_.begin());
  std::copy(std::begin(sp::spi_rom_data_80), std::end(sp::spi_rom_data_80),
            spi_rom_user_data_.begin());

  // generate a random 11-digit serial number into the factory ROM serial region
  // (bytes 0x00-0x0F); zero-pad the remainder of the 16-byte field. A first byte
  // < 0x80 (ASCII digits are 0x30-0x39) signals "serial present".
  static constexpr size_t serial_digits = 11;
  for (size_t i = 0; i < serial_digits; ++i)
    spi_rom_factory_data_[i] = static_cast<uint8_t>('0' + (esp_random() % 10));
  std::fill(spi_rom_factory_data_.begin() + serial_digits, spi_rom_factory_data_.begin() + 16,
            0x00);

  // use the ESP32's factory MAC as the controller's BT MAC address
  esp_read_mac(mac_address_.data(), ESP_MAC_WIFI_STA);

  // set the report's housekeeping fields (battery / connection / powered)
  std::lock_guard<std::recursive_mutex> lock(input_report_mutex_);
  apply_housekeeping();
}

void SwitchPro::apply_housekeeping() {
  // called with input_report_mutex_ held
  input_report_.set_usb_powered(true);
  input_report_.set_battery_charging(true);
  input_report_.set_battery_level(static_cast<float>(battery_level_));
  input_report_.set_connection_info(sp::PRO_CONTROLLER.connection_info);
}

void SwitchPro::update_input_report(const std::function<void(InputReport &)> &fn) {
  std::lock_guard<std::recursive_mutex> lock(input_report_mutex_);
  if (fn)
    fn(input_report_);
  // re-apply housekeeping so a caller-issued reset() does not clear it
  apply_housekeeping();
}

std::vector<uint8_t> SwitchPro::get_input_report() const {
  if (!hid_ready_)
    return {};
  std::lock_guard<std::recursive_mutex> lock(input_report_mutex_);
  auto report = input_report_.get_report();
  // Finalize the streamed 0x30 standard report with the current protocol state
  // (the subcommand-reply path does this for 0x21 replies, but the streamed path
  // must too): the vibrator byte, and the IMU frames when the host enabled the
  // IMU. Both fields come from atomics written by the TinyUSB subcommand handlers.
  if (report.size() > 11)
    report[11] = vibrator_report_;
  set_imu_data(report); // no-op unless imu_enabled_
  return report;
}

void SwitchPro::set_battery_level(uint8_t level) {
  std::lock_guard<std::recursive_mutex> lock(input_report_mutex_);
  battery_level_ = level;
  input_report_.set_battery_level(static_cast<float>(level));
}

void SwitchPro::set_trigger_elapsed_times(const std::array<uint16_t, 7> &times_10ms) {
  std::lock_guard<std::recursive_mutex> lock(input_report_mutex_);
  for (size_t i = 0; i < times_10ms.size(); ++i)
    trigger_times_.values[i] = times_10ms[i];
}

std::vector<uint8_t> SwitchPro::device_info_report() const {
  // the device-init report data (device type + placeholder MAC) with our MAC
  // stamped in. The protocol encodes the MAC in REVERSED byte order (the
  // placeholder in device_init_report_data is reversed), while esp_read_mac()
  // returns forward order -- so reverse it here.
  std::vector<uint8_t> data(sp::device_init_report_data,
                            sp::device_init_report_data + std::size(sp::device_init_report_data));
  std::reverse_copy(mac_address_.begin(), mac_address_.end(),
                    data.begin() + sp::device_init_report_data_mac_addr_offset);
  return data;
}

std::optional<SwitchPro::ReportData> SwitchPro::on_attach() {
  // A (re)attach starts a fresh handshake: reset session readiness/state so the
  // sender does not stream stale 0x30 input reports before the new 0x81/0x80
  // exchange completes (the example uses this as its mount callback, and there is
  // no separate detach reset).
  hid_ready_ = false;
  // Per-session state set by subcommand handlers must reset too, or the first
  // streamed reports after a reconnect carry stale state (e.g. IMU frames before
  // the new host enables the IMU).
  imu_enabled_ = false;
  vibrator_report_ = 0;
  {
    std::lock_guard<std::recursive_mutex> lock(input_report_mutex_);
    input_report_mode_ = 0;
    input_report_id_ = 0x21;
    vibration_enabled_ = false;
    player_number_ = 0;
  }
  // kick off the initialization sequence by advertising device info (report 0x81)
  return ReportData{sp::DEVICE_INIT_REPORT, device_info_report()};
}

std::optional<SwitchPro::ReportData> SwitchPro::on_hid_report(uint8_t report_id,
                                                              const uint8_t *data, size_t len) {
  (void)report_id; // the report "type" is data[0], not the HID report id
  if (data == nullptr || len == 0)
    return std::nullopt;

  using namespace sp;

  switch (data[0]) {
  case HOST_INIT_REPORT: {
    if (len < 2)
      return std::nullopt;
    uint8_t cmd = data[1];
    std::vector<uint8_t> resp(sp::REPORT_SIZE, 0);
    resp[0] = cmd;
    switch (cmd) {
    case INIT_COMMAND_DEVICE_INFO:
      // reply with the device type + MAC (a bare 0x81 0x01 with no info is
      // rejected by the host) -- the same payload on_attach() advertises.
      resp = device_info_report();
      break;
    case INIT_COMMAND_HANDSHAKE: {
      // echo the payload back to the host, bounded to the response buffer (len is
      // caller-supplied, so an oversized report must not overflow resp).
      const size_t n = std::min<size_t>(len - 1, resp.size());
      std::copy(data + 1, data + 1 + n, resp.begin());
      break;
    }
    case INIT_COMMAND_SET_BAUD_RATE:
      break;
    case INIT_COMMAND_ENABLE_USB_HID:
      // ok to start sending input reports
      hid_ready_ = true;
      break;
    case INIT_COMMAND_ENABLE_BT_HID:
      // switches back to BT; nothing to do for the USB emulation
      break;
    default:
      logger_.debug("Unknown init command: 0x{:02x}", cmd);
      break;
    }
    return ReportData{DEVICE_INIT_REPORT, std::move(resp)};
  }
  case HOST_OUTPUT_REPORT:
    return process_command(data, len);
  case HOST_RUMBLE_REPORT:
    // TODO: process the rumble packet (no reply required)
    return std::nullopt;
  default:
    logger_.debug("Unhandled host report type: 0x{:02x}", data[0]);
    return std::nullopt;
  }
}
