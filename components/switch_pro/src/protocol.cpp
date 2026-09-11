#include "switch_pro.hpp"

#include <algorithm>
#include <cstring>

#include <esp_random.h>

using namespace espp;
using namespace sp;

// credits to https://github.com/Brikwerk/nxbt/blob/master/nxbt/controller/protocol.py
// for the reference Joy-Con / Switch Pro controller protocol implementation.
//
// SWITCH blocks on 0x81 0x01 and 0x21 0x03.

// Copy `count` bytes from `src` into `arr` starting at `start` (bounded to arr).
static void replace_subarray(std::vector<uint8_t> &arr, size_t start, size_t count,
                             const uint8_t *src) {
  for (size_t i = 0; i < count && (start + i) < arr.size(); ++i)
    arr[start + i] = src[i];
}

SwitchPro::ReportData SwitchPro::process_command(const uint8_t *data, size_t len) {
  // Parse the Switch's message
  Message message(data, len);

  // Prep the most common response, which contains the full input report
  std::vector<uint8_t> report;
  {
    std::lock_guard<std::recursive_mutex> lock(input_report_mutex_);
    report = input_report_.get_report();
  }

  report[12] = 0x80;
  report[13] = message.subcommand_id;
  report[14] = 0; // sane default

  // Respond to the parsed message
  switch (message.response) {
  case Response::ONLY_CONTROLLER_STATE:
    set_subcommand_reply(report);
    report[12] = 0x80; // ACK byte
    report[13] = 0x00; // subcommand reply
    break;
  case Response::BT_MANUAL_PAIRING:
    set_subcommand_reply(report);
    report[12] = 0x81; // ACK byte
    report[13] = 0x01; // subcommand reply
    break;
  case Response::REQUEST_DEVICE_INFO:
    hid_ready_ = true;
    set_subcommand_reply(report);
    set_device_info(report);
    break;
  case Response::SET_SHIPMENT:
    set_subcommand_reply(report);
    set_shipment(report);
    break;
  case Response::SPI_READ:
    set_subcommand_reply(report);
    spi_read(report, message);
    break;
  case Response::SET_MODE:
    set_subcommand_reply(report);
    set_mode(report, message);
    break;
  case Response::TRIGGER_BUTTONS_ELAPSED:
    set_subcommand_reply(report);
    set_trigger_buttons(report);
    break;
  case Response::TOGGLE_IMU:
    set_subcommand_reply(report);
    toggle_imu(report, message);
    break;
  case Response::ENABLE_VIBRATION:
    set_subcommand_reply(report);
    enable_vibration(report);
    break;
  case Response::SET_PLAYER:
    set_subcommand_reply(report);
    set_player_lights(report, message);
    break;
  case Response::SET_NFC_IR_STATE:
    set_subcommand_reply(report);
    set_nfc_ir_state(report);
    break;
  case Response::SET_NFC_IR_CONFIG:
    set_subcommand_reply(report);
    set_nfc_ir_config(report);
    break;
  // Bad / unhandled packets: ignore the subcommand rather than NACK, so we do not
  // get stuck arguing with the Switch (a NACK loop).
  default:
    set_unknown_subcommand(report, message.subcommand_id);
    break;
  }

  return {input_report_id_, report};
}

void SwitchPro::set_subcommand_reply(std::vector<uint8_t> &report) {
  input_report_id_ = 0x21; // subcommand reply input report id

  // The vibrator byte seems to change when a subcommand reply is sent; emulate
  // that by picking a random one from the known set.
  const std::size_t max_index = sizeof(sp::vibrator_bytes) / sizeof(sp::vibrator_bytes[0]);
  vibrator_report_ = sp::vibrator_bytes[esp_random() % max_index];

  set_standard_input_report(report);
}

void SwitchPro::set_unknown_subcommand(std::vector<uint8_t> &report, uint8_t subcommand_id) {
  report[12] = 0x80;          // ACK
  report[13] = subcommand_id; // unknown subcommand id
  report[14] = 0x03;          // unknown subcommand reply
}

void SwitchPro::set_standard_input_report(std::vector<uint8_t> &report) {
  {
    std::lock_guard<std::recursive_mutex> lock(input_report_mutex_);
    report[0] = input_report_.get_counter();
  }
  if (hid_ready_) {
    // the gamepad bytes (1-11) are already correct; just set the vibrator byte
    report[11] = vibrator_report_;
  }
}

void SwitchPro::set_device_info(std::vector<uint8_t> &report) {
  report[12] = 0x82; // ACK reply
  report[13] = 0x02; // subcommand reply

  replace_subarray(report, 14, sizeof(sp::device_info), sp::device_info);

  // overwrite the device-info MAC placeholder (bytes 18-23) with our MAC
  std::memcpy(report.data() + 18, mac_address_.data(), mac_address_.size());
}

void SwitchPro::set_shipment(std::vector<uint8_t> &report) {
  report[12] = 0x80; // ACK reply
  report[13] = 0x08; // subcommand reply
}

void SwitchPro::toggle_imu(std::vector<uint8_t> &report, sp::Message &message) {
  imu_enabled_ = (message.subcommand_len > 1 && message.subcommand[1] == 0x01);
  report[12] = 0x80; // ACK reply
  report[13] = 0x40; // subcommand reply
}

void SwitchPro::set_imu_data(std::vector<uint8_t> &report) {
  if (!imu_enabled_)
    return;
  static constexpr uint8_t imu_data[] = {0x75, 0xFD, 0xFD, 0xFF, 0x09, 0x10, 0x21, 0x00, 0xD5,
                                         0xFF, 0xE0, 0xFF, 0x72, 0xFD, 0xF9, 0xFF, 0x0A, 0x10,
                                         0x22, 0x00, 0xD5, 0xFF, 0xE0, 0xFF, 0x76, 0xFD, 0xFC,
                                         0xFF, 0x09, 0x10, 0x23, 0x00, 0xD5, 0xFF, 0xE0, 0xFF};
  replace_subarray(report, 12, sizeof(imu_data), imu_data);
}

uint8_t SwitchPro::spi_read_impl(uint8_t bank, uint8_t reg, uint8_t read_length,
                                 uint8_t *response) {
  using namespace sp;
  auto read_from = [&](const uint8_t *src, size_t src_size) -> uint8_t {
    // clamp to the source array so a malformed / out-of-range request cannot
    // over-read the ROM blob.
    if (reg >= src_size)
      return 0;
    const size_t avail = src_size - reg;
    const uint8_t n = static_cast<uint8_t>(std::min<size_t>(read_length, avail));
    std::memcpy(response, src + reg, n);
    return n;
  };
  if (bank == REG_BANK_SHIPMENT) {
    // shipment bank: return zeros (no shipment data to report)
    std::fill(response, response + read_length, 0);
    return read_length;
  } else if (bank == REG_BANK_FACTORY_CONFIG) {
    return read_from(spi_rom_factory_data_.data(), spi_rom_factory_data_.size());
  } else if (bank == REG_BANK_USER_CAL) {
    return read_from(spi_rom_user_data_.data(), spi_rom_user_data_.size());
  }
  return 0;
}

void SwitchPro::spi_read(std::vector<uint8_t> &report, sp::Message &message) {
  if (message.subcommand_len < 6) {
    // malformed SPI read request: NACK
    report[12] = 0x83;
    report[13] = 0x00;
    return;
  }
  uint8_t addr_top = message.subcommand[2];
  uint8_t addr_bottom = message.subcommand[1];
  uint8_t read_length = message.subcommand[5];

  // guard the destination: the SPI header occupies bytes 12-18, data starts at 19
  if (read_length > report.size() - 19)
    read_length = static_cast<uint8_t>(report.size() - 19);

  const uint8_t read = spi_read_impl(addr_top, addr_bottom, read_length, report.data() + 19);
  if (read > 0) {
    report[12] = 0x90;        // ACK byte
    report[13] = 0x10;        // subcommand reply
    report[14] = addr_bottom; // read address (low)
    report[15] = addr_top;    // read address (high)
    report[16] = 0;
    report[17] = 0;
    report[18] = read; // bytes actually read
    return;
  }

  // read failed: NACK
  report[12] = 0x83;
  report[13] = 0x00;
}

void SwitchPro::set_mode(std::vector<uint8_t> &report, sp::Message &message) {
  report[12] = 0x80; // ACK byte
  report[13] = 0x03; // subcommand reply
  if (message.subcommand_len > 1)
    input_report_mode_ = message.subcommand[1]; // 0x30 std, 0x31 nfc/ir, 0x3F simple
}

void SwitchPro::set_trigger_buttons(std::vector<uint8_t> &report) {
  report[12] = 0x83; // ACK byte
  report[13] = 0x04; // subcommand reply
  // 7 little-endian uint16 in units of 10ms: L,R,ZL,ZR,SL,SR,HOME. See
  // https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering
  std::memcpy(report.data() + 14, &trigger_times_, sizeof(trigger_times_));
}

void SwitchPro::enable_vibration(std::vector<uint8_t> &report) {
  report[12] = 0x82; // ACK reply
  report[13] = 0x48; // subcommand reply
  vibration_enabled_ = true;
}

void SwitchPro::set_player_lights(std::vector<uint8_t> &report, sp::Message &message) {
  report[12] = 0x80; // ACK byte
  report[13] = 0x30; // subcommand reply

  const uint8_t bitfield = (message.subcommand_len > 1) ? message.subcommand[1] : 0;
  if (bitfield == 0x01 || bitfield == 0x10)
    player_number_ = 1;
  else if (bitfield == 0x03 || bitfield == 0x30)
    player_number_ = 2;
  else if (bitfield == 0x07 || bitfield == 0x70)
    player_number_ = 3;
  else if (bitfield == 0x0F || bitfield == 0xF0)
    player_number_ = 4;
}

void SwitchPro::set_nfc_ir_state(std::vector<uint8_t> &report) {
  report[12] = 0x80; // ACK byte
  report[13] = 0x22; // subcommand reply
}

void SwitchPro::set_nfc_ir_config(std::vector<uint8_t> &report) {
  report[12] = 0xA0; // ACK byte
  report[13] = 0x21; // subcommand reply

  // NFC/IR state data (8 bytes at offset 14). NOTE: the reference had a swapped
  // (start, end) argument here that copied nothing; write the 8 bytes correctly.
  static constexpr uint8_t params[] = {0x01, 0x00, 0xFF, 0x00, 0x08, 0x00, 0x1B, 0x01};
  replace_subarray(report, 14, sizeof(params), params);
  report[47] = 0xC8;
}
