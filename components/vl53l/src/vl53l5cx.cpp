// The sensor sequences in this file (reboot, firmware upload, NVM / crosstalk
// loading, the DCI command interface and the result block parsing) are a port
// of the STMicroelectronics VL53L5CX Ultra Lite Driver, API revision
// VL53L5CX_1.1.2 (vl53l5cx_api.c, vl53l5cx_plugin_motion_indicator.c):
//
// Copyright (c) 2021 STMicroelectronics
// Redistributed under the BSD-3-Clause license; see vl53l5cx_firmware.cpp for
// the full text.

#include "vl53l5cx.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

#include "vl53l5cx_firmware.hpp"

using namespace espp;

namespace {
uint32_t load_u32(const uint8_t *p) {
  uint32_t v;
  std::memcpy(&v, p, sizeof(v));
  return v;
}
void store_u32(uint8_t *p, uint32_t v) { std::memcpy(p, &v, sizeof(v)); }
} // namespace

Vl53l5cx::Vl53l5cx(const Config &config)
    : Base({}, "Vl53l5cx", config.log_level) {
  set_address(config.device_address);
  set_write(config.write);
  set_read(config.read);
  if (config.auto_init) {
    std::error_code ec;
    if (!initialize(ec)) {
      logger_.error("Initialization failed: {}", ec.message());
    }
  }
}

const char *Vl53l5cx::target_status_name(TargetStatus status) {
  switch (status) {
  case TargetStatus::NOT_UPDATED:
    return "not updated";
  case TargetStatus::SIGNAL_RATE_TOO_LOW_SPAD_ARRAY:
    return "signal rate too low on the SPAD array";
  case TargetStatus::TARGET_PHASE:
    return "target phase";
  case TargetStatus::SIGMA_TOO_HIGH:
    return "sigma estimator too high";
  case TargetStatus::TARGET_CONSISTENCY_FAILED:
    return "target consistency failed";
  case TargetStatus::RANGE_VALID:
    return "range valid";
  case TargetStatus::WRAP_AROUND_NOT_PERFORMED:
    return "wrap-around not performed";
  case TargetStatus::RATE_CONSISTENCY_FAILED:
    return "rate consistency failed";
  case TargetStatus::SIGNAL_RATE_TOO_LOW_TARGET:
    return "signal rate too low for the target";
  case TargetStatus::RANGE_VALID_LARGE_PULSE:
    return "range valid (large pulse)";
  case TargetStatus::RANGE_VALID_NO_PREVIOUS_TARGET:
    return "range valid (no previous target)";
  case TargetStatus::MEASUREMENT_CONSISTENCY_FAILED:
    return "measurement consistency failed";
  case TargetStatus::TARGET_BLURRED:
    return "target blurred by another one";
  case TargetStatus::TARGET_INCONSISTENT:
    return "target inconsistent";
  case TargetStatus::NO_TARGET:
    return "no target";
  }
  return "unknown";
}

// ---------------------------------------------------------------------------
// Raw register access
// ---------------------------------------------------------------------------

bool Vl53l5cx::write_byte(uint16_t reg, uint8_t value, std::error_code &ec) {
  return write_multi(reg, &value, 1, ec);
}

bool Vl53l5cx::read_byte(uint16_t reg, uint8_t &value, std::error_code &ec) {
  return read_multi(reg, &value, 1, ec);
}

bool Vl53l5cx::write_multi(uint16_t reg, const uint8_t *data, size_t length, std::error_code &ec) {
  // the register address auto-increments, so a long write (the firmware image)
  // can go out in bounded transactions; the base class stages each one on the
  // stack
  for (size_t offset = 0; offset < length; offset += MAX_WRITE_CHUNK) {
    const size_t chunk = std::min(MAX_WRITE_CHUNK, length - offset);
    write_many_to_register(static_cast<uint16_t>(reg + offset), data + offset, chunk, ec);
    if (ec) {
      logger_.error("Write of {} bytes to 0x{:04x} failed: {}", chunk, reg + offset, ec.message());
      return false;
    }
  }
  return true;
}

bool Vl53l5cx::read_multi(uint16_t reg, uint8_t *data, size_t length, std::error_code &ec) {
  read_many_from_register(reg, data, length, ec);
  if (ec) {
    logger_.error("Read of {} bytes from 0x{:04x} failed: {}", length, reg, ec.message());
    return false;
  }
  return true;
}

bool Vl53l5cx::poll_for_answer(size_t size, size_t pos, uint16_t reg, uint8_t mask,
                               uint8_t expected, std::error_code &ec) {
  for (int attempt = 0; attempt <= POLL_ATTEMPTS; ++attempt) {
    if (!read_multi(reg, temp_buffer_.data(), size, ec)) {
      return false;
    }
    // byte 2 of a 4-byte command status is the firmware's error code
    if (size >= 4 && temp_buffer_[2] >= 0x7f) {
      logger_.error("Firmware error 0x{:02x} while polling 0x{:04x}", temp_buffer_[2], reg);
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    if ((temp_buffer_[pos] & mask) == expected) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(POLL_INTERVAL_MS));
  }
  logger_.error("Timed out polling 0x{:04x} for 0x{:02x} (mask 0x{:02x}); last 0x{:02x}", reg,
                expected, mask, temp_buffer_[pos]);
  ec = std::make_error_code(std::errc::timed_out);
  return false;
}

void Vl53l5cx::swap_buffer(uint8_t *buffer, size_t size) {
  for (size_t i = 0; i + 4 <= size; i += 4) {
    std::swap(buffer[i], buffer[i + 3]);
    std::swap(buffer[i + 1], buffer[i + 2]);
  }
}

// ---------------------------------------------------------------------------
// DCI: the firmware's parameter interface
// ---------------------------------------------------------------------------

bool Vl53l5cx::dci_read_data(uint8_t *data, uint16_t index, uint16_t size, std::error_code &ec) {
  const size_t read_size = static_cast<size_t>(size) + 12;
  if (read_size > TEMP_BUFFER_SIZE) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  const uint8_t cmd[12] = {static_cast<uint8_t>(index >> 8),
                           static_cast<uint8_t>(index & 0xff),
                           static_cast<uint8_t>((size & 0xff0) >> 4),
                           static_cast<uint8_t>((size & 0xf) << 4),
                           0x00,
                           0x00,
                           0x00,
                           0x0f,
                           0x00,
                           0x02,
                           0x00,
                           0x08};
  // request the data, wait for the firmware, read it back (4-byte header +
  // data + 8-byte footer, big-endian words)
  if (!write_multi(UI_CMD_END - 11, cmd, sizeof(cmd), ec) ||
      !poll_for_answer(4, 1, UI_CMD_STATUS, 0xff, 0x03, ec) ||
      !read_multi(UI_CMD_START, temp_buffer_.data(), read_size, ec)) {
    return false;
  }
  swap_buffer(temp_buffer_.data(), read_size);
  std::memcpy(data, temp_buffer_.data() + 4, size);
  return true;
}

bool Vl53l5cx::dci_write_data(const uint8_t *data, uint16_t index, uint16_t size,
                              std::error_code &ec) {
  const size_t write_size = static_cast<size_t>(size) + 12;
  if (write_size > TEMP_BUFFER_SIZE) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  const uint8_t header[4] = {static_cast<uint8_t>(index >> 8), static_cast<uint8_t>(index & 0xff),
                             static_cast<uint8_t>((size & 0xff0) >> 4),
                             static_cast<uint8_t>((size & 0xf) << 4)};
  const uint16_t footer_size = static_cast<uint16_t>(size + 8);
  const uint8_t footer[8] = {0x00,
                             0x00,
                             0x00,
                             0x0f,
                             0x05,
                             0x01,
                             static_cast<uint8_t>(footer_size >> 8),
                             static_cast<uint8_t>(footer_size & 0xff)};
  std::memcpy(temp_buffer_.data(), header, sizeof(header));
  std::memcpy(temp_buffer_.data() + 4, data, size);
  swap_buffer(temp_buffer_.data() + 4, size);
  std::memcpy(temp_buffer_.data() + 4 + size, footer, sizeof(footer));
  const uint16_t address = static_cast<uint16_t>(UI_CMD_END - write_size + 1);
  return write_multi(address, temp_buffer_.data(), write_size, ec) &&
         poll_for_answer(4, 1, UI_CMD_STATUS, 0xff, 0x03, ec);
}

bool Vl53l5cx::dci_replace_data(uint16_t index, uint16_t size, const uint8_t *new_data,
                                uint16_t new_size, uint16_t new_pos, std::error_code &ec) {
  uint8_t buffer[32];
  if (size > sizeof(buffer) || new_pos + new_size > size) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  if (!dci_read_data(buffer, index, size, ec)) {
    return false;
  }
  std::memcpy(buffer + new_pos, new_data, new_size);
  return dci_write_data(buffer, index, size, ec);
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

bool Vl53l5cx::is_alive(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  uint8_t device_id = 0;
  uint8_t revision_id = 0;
  if (!write_byte(0x7fff, 0x00, ec) || !read_byte(0x0000, device_id, ec) ||
      !read_byte(0x0001, revision_id, ec) || !write_byte(0x7fff, 0x02, ec)) {
    return false;
  }
  const bool alive = device_id == DEVICE_ID && revision_id == REVISION_ID;
  if (!alive) {
    logger_.warn("Device id 0x{:02x} / revision 0x{:02x} is not a VL53L5CX (0x{:02x} / 0x{:02x})",
                 device_id, revision_id, DEVICE_ID, REVISION_ID);
  }
  return alive;
}

bool Vl53l5cx::initialize(std::error_code &ec) {
  using namespace std::chrono_literals;
  using namespace vl53l5cx_detail;
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  initialized_ = false;
  ranging_ = false;
  motion_enabled_ = false;

  if (!is_alive(ec) && ec) {
    return false;
  }

  // the sequence below is the ULD's vl53l5cx_init(); the register writes are
  // undocumented (ST's "boot" and "wake the MCU" recipes)
  struct Step {
    uint16_t reg;
    uint8_t value;
  };
  auto run = [&](const Step *steps, size_t count) {
    for (size_t i = 0; i < count; ++i) {
      if (!write_byte(steps[i].reg, steps[i].value, ec)) {
        return false;
      }
    }
    return true;
  };

  // software reboot
  static constexpr Step kReboot1[] = {
      {0x7fff, 0x00}, {0x0009, 0x04}, {0x000F, 0x40}, {0x000A, 0x03}};
  if (!run(kReboot1, std::size(kReboot1))) {
    return false;
  }
  uint8_t tmp = 0;
  if (!read_byte(0x7FFF, tmp, ec)) {
    return false;
  }
  static constexpr Step kReboot2[] = {
      {0x000C, 0x01}, {0x0101, 0x00}, {0x0102, 0x00}, {0x010A, 0x01}, {0x4002, 0x01},
      {0x4002, 0x00}, {0x010A, 0x03}, {0x0103, 0x01}, {0x000C, 0x00}, {0x000F, 0x43}};
  if (!run(kReboot2, std::size(kReboot2))) {
    return false;
  }
  std::this_thread::sleep_for(1ms);
  static constexpr Step kReboot3[] = {{0x000F, 0x40}, {0x000A, 0x01}};
  if (!run(kReboot3, std::size(kReboot3))) {
    return false;
  }
  std::this_thread::sleep_for(100ms);

  // wait for the sensor to boot
  if (!write_byte(0x7fff, 0x00, ec) || !poll_for_answer(1, 0, 0x06, 0xff, 1, ec) ||
      !write_byte(0x000E, 0x01, ec) || !write_byte(0x7fff, 0x02, ec)) {
    logger_.error("The sensor did not boot");
    return false;
  }
  // enable firmware access
  if (!write_byte(0x03, 0x0D, ec) || !write_byte(0x7fff, 0x01, ec) ||
      !poll_for_answer(1, 0, 0x21, 0x10, 0x10, ec) || !write_byte(0x7fff, 0x00, ec)) {
    return false;
  }
  // enable host access to GO1, power-on status
  static constexpr Step kPowerOn[] = {
      {0x0C, 0x01},   {0x7fff, 0x00}, {0x101, 0x00}, {0x102, 0x00},  {0x010A, 0x01}, {0x4002, 0x01},
      {0x4002, 0x00}, {0x010A, 0x03}, {0x103, 0x01}, {0x400F, 0x00}, {0x21A, 0x43},  {0x21A, 0x03},
      {0x21A, 0x01},  {0x21A, 0x00},  {0x219, 0x00}, {0x21B, 0x00}};
  if (!run(kPowerOn, std::size(kPowerOn))) {
    return false;
  }
  // wake up the MCU
  static constexpr Step kWakeMcu[] = {
      {0x7fff, 0x00}, {0x0C, 0x00}, {0x7fff, 0x01}, {0x20, 0x07}, {0x20, 0x06}};
  if (!run(kWakeMcu, std::size(kWakeMcu))) {
    return false;
  }

  // upload the firmware: three RAM pages (0x7fff selects the page)
  logger_.info("Uploading the VL53L5CX firmware ({} bytes)", kFirmwareSize);
  if (!write_byte(0x7fff, 0x09, ec) || !write_multi(0, kFirmware, 0x8000, ec) ||
      !write_byte(0x7fff, 0x0a, ec) || !write_multi(0, kFirmware + 0x8000, 0x8000, ec) ||
      !write_byte(0x7fff, 0x0b, ec) || !write_multi(0, kFirmware + 0x10000, 0x5000, ec) ||
      !write_byte(0x7fff, 0x01, ec)) {
    logger_.error("Firmware upload failed");
    return false;
  }
  // check the firmware was taken
  if (!write_byte(0x7fff, 0x02, ec) || !write_byte(0x03, 0x0D, ec) ||
      !write_byte(0x7fff, 0x01, ec) || !poll_for_answer(1, 0, 0x21, 0x10, 0x10, ec) ||
      !write_byte(0x7fff, 0x00, ec) || !write_byte(0x0C, 0x01, ec)) {
    logger_.error("The firmware did not start");
    return false;
  }
  // reset the MCU and wait for it to boot
  static constexpr Step kResetMcu[] = {{0x7FFF, 0x00}, {0x114, 0x00}, {0x115, 0x00}, {0x116, 0x42},
                                       {0x117, 0x00},  {0x0B, 0x00},  {0x0C, 0x00},  {0x0B, 0x01}};
  if (!run(kResetMcu, std::size(kResetMcu)) || !poll_for_answer(1, 0, 0x06, 0xff, 0x00, ec) ||
      !write_byte(0x7fff, 0x02, ec)) {
    logger_.error("The MCU did not boot");
    return false;
  }

  // read the NVM (per-unit offset calibration) into the offset buffer
  if (!write_multi(0x2fd8, kGetNvmCommand, kGetNvmCommandSize, ec) ||
      !poll_for_answer(4, 0, UI_CMD_STATUS, 0xff, 2, ec) ||
      !read_multi(UI_CMD_START, temp_buffer_.data(), NVM_DATA_SIZE, ec)) {
    logger_.error("Reading the NVM data failed");
    return false;
  }
  std::memcpy(offset_data_.data(), temp_buffer_.data(), OFFSET_BUFFER_SIZE);
  if (!send_offset_data(Resolution::ZONES_4X4, ec)) {
    return false;
  }
  // default crosstalk shape
  std::memcpy(xtalk_data_.data(), kDefaultXtalk, XTALK_BUFFER_SIZE);
  if (!send_xtalk_data(Resolution::ZONES_4X4, ec)) {
    return false;
  }
  // default configuration
  if (!write_multi(0x2c34, kDefaultConfiguration, kDefaultConfigurationSize, ec) ||
      !poll_for_answer(4, 1, UI_CMD_STATUS, 0xff, 0x03, ec)) {
    logger_.error("Sending the default configuration failed");
    return false;
  }
  const uint8_t pipe_control[4] = {TARGETS_PER_ZONE, 0x00, 0x01, 0x00};
  uint8_t single_range[4];
  store_u32(single_range, 0x01);
  if (!dci_write_data(pipe_control, DCI_PIPE_CONTROL, sizeof(pipe_control), ec) ||
      !dci_write_data(single_range, DCI_SINGLE_RANGE, sizeof(single_range), ec)) {
    return false;
  }
  resolution_ = Resolution::ZONES_4X4;
  initialized_ = true;
  logger_.info("Initialized (4x4, autonomous, 1 Hz)");
  return true;
}

bool Vl53l5cx::send_offset_data(Resolution resolution, std::error_code &ec) {
  static constexpr uint8_t kDss4x4[] = {0x0F, 0x04, 0x04, 0x00, 0x08, 0x10, 0x10, 0x07};
  static constexpr uint8_t kFooter[] = {0x00, 0x00, 0x00, 0x0F, 0x03, 0x01, 0x01, 0xE4};
  std::memcpy(temp_buffer_.data(), offset_data_.data(), OFFSET_BUFFER_SIZE);
  // the NVM holds 8x8 data: average 2x2 blocks for 4x4
  if (resolution == Resolution::ZONES_4X4) {
    std::memcpy(temp_buffer_.data() + 0x10, kDss4x4, sizeof(kDss4x4));
    swap_buffer(temp_buffer_.data(), OFFSET_BUFFER_SIZE);
    uint32_t signal_grid[64];
    int16_t range_grid[64];
    std::memcpy(signal_grid, temp_buffer_.data() + 0x3C, sizeof(signal_grid));
    std::memcpy(range_grid, temp_buffer_.data() + 0x140, sizeof(range_grid));
    for (int j = 0; j < 4; j++) {
      for (int i = 0; i < 4; i++) {
        const int k = (2 * i) + (16 * j);
        signal_grid[i + (4 * j)] =
            (signal_grid[k] + signal_grid[k + 1] + signal_grid[k + 8] + signal_grid[k + 9]) / 4;
        range_grid[i + (4 * j)] = static_cast<int16_t>(
            (range_grid[k] + range_grid[k + 1] + range_grid[k + 8] + range_grid[k + 9]) / 4);
      }
    }
    std::memset(&range_grid[0x10], 0, 96);
    std::memset(&signal_grid[0x10], 0, 192);
    std::memcpy(temp_buffer_.data() + 0x3C, signal_grid, sizeof(signal_grid));
    std::memcpy(temp_buffer_.data() + 0x140, range_grid, sizeof(range_grid));
    swap_buffer(temp_buffer_.data(), OFFSET_BUFFER_SIZE);
  }
  // drop the 8-byte NVM header (the ULD shifts the whole buffer)
  for (size_t k = 0; k < OFFSET_BUFFER_SIZE - 4; k++) {
    temp_buffer_[k] = temp_buffer_[k + 8];
  }
  std::memcpy(temp_buffer_.data() + 0x1E0, kFooter, sizeof(kFooter));
  if (!write_multi(0x2e18, temp_buffer_.data(), OFFSET_BUFFER_SIZE, ec) ||
      !poll_for_answer(4, 1, UI_CMD_STATUS, 0xff, 0x03, ec)) {
    logger_.error("Sending the offset data failed");
    return false;
  }
  return true;
}

bool Vl53l5cx::send_xtalk_data(Resolution resolution, std::error_code &ec) {
  static constexpr uint8_t kRes4x4[] = {0x0F, 0x04, 0x04, 0x17, 0x08, 0x10, 0x10, 0x07};
  static constexpr uint8_t kDss4x4[] = {0x00, 0x78, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08};
  static constexpr uint8_t kProfile4x4[] = {0xA0, 0xFC, 0x01, 0x00};
  std::memcpy(temp_buffer_.data(), xtalk_data_.data(), XTALK_BUFFER_SIZE);
  if (resolution == Resolution::ZONES_4X4) {
    std::memcpy(temp_buffer_.data() + 0x8, kRes4x4, sizeof(kRes4x4));
    std::memcpy(temp_buffer_.data() + 0x020, kDss4x4, sizeof(kDss4x4));
    swap_buffer(temp_buffer_.data(), XTALK_BUFFER_SIZE);
    uint32_t signal_grid[64];
    std::memcpy(signal_grid, temp_buffer_.data() + 0x34, sizeof(signal_grid));
    for (int j = 0; j < 4; j++) {
      for (int i = 0; i < 4; i++) {
        const int k = (2 * i) + (16 * j);
        signal_grid[i + (4 * j)] =
            (signal_grid[k] + signal_grid[k + 1] + signal_grid[k + 8] + signal_grid[k + 9]) / 4;
      }
    }
    std::memset(&signal_grid[0x10], 0, 192);
    std::memcpy(temp_buffer_.data() + 0x34, signal_grid, sizeof(signal_grid));
    swap_buffer(temp_buffer_.data(), XTALK_BUFFER_SIZE);
    std::memcpy(temp_buffer_.data() + 0x134, kProfile4x4, sizeof(kProfile4x4));
    std::memset(temp_buffer_.data() + 0x078, 0, 4);
  }
  if (!write_multi(0x2cf8, temp_buffer_.data(), XTALK_BUFFER_SIZE, ec) ||
      !poll_for_answer(4, 1, UI_CMD_STATUS, 0xff, 0x03, ec)) {
    logger_.error("Sending the crosstalk data failed");
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------

bool Vl53l5cx::refuse_while_ranging(const char *what, std::error_code &ec) {
  if (ranging_) {
    logger_.error("Cannot change the {} while ranging; stop ranging first", what);
    ec = std::make_error_code(std::errc::device_or_resource_busy);
    return true;
  }
  return false;
}

bool Vl53l5cx::set_i2c_address(uint8_t address, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (address > 0x7f) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  if (!write_byte(0x7fff, 0x00, ec) || !write_byte(0x0004, address, ec)) {
    return false;
  }
  // from here on the sensor only answers on the new address
  set_address(address);
  return write_byte(0x7fff, 0x02, ec);
}

Vl53l5cx::PowerMode Vl53l5cx::get_power_mode(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  uint8_t tmp = 0;
  if (!write_byte(0x7FFF, 0x00, ec) || !read_byte(0x0009, tmp, ec)) {
    return PowerMode::WAKEUP;
  }
  PowerMode mode = PowerMode::WAKEUP;
  switch (tmp) {
  case 0x04:
    mode = PowerMode::WAKEUP;
    break;
  case 0x02:
    mode = PowerMode::SLEEP;
    break;
  default:
    logger_.error("Unknown power mode register value 0x{:02x}", tmp);
    ec = std::make_error_code(std::errc::io_error);
    break;
  }
  std::error_code restore_ec;
  write_byte(0x7FFF, 0x02, restore_ec);
  if (restore_ec && !ec) {
    ec = restore_ec;
  }
  return mode;
}

bool Vl53l5cx::set_power_mode(PowerMode mode, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (refuse_while_ranging("power mode", ec)) {
    return false;
  }
  const PowerMode current = get_power_mode(ec);
  if (ec) {
    return false;
  }
  if (current == mode) {
    return true;
  }
  bool ok = write_byte(0x7FFF, 0x00, ec);
  if (ok) {
    if (mode == PowerMode::WAKEUP) {
      ok = write_byte(0x09, 0x04, ec) && poll_for_answer(1, 0, 0x06, 0x01, 1, ec);
    } else {
      ok = write_byte(0x09, 0x02, ec) && poll_for_answer(1, 0, 0x06, 0x01, 0, ec);
    }
  }
  std::error_code restore_ec;
  write_byte(0x7FFF, 0x02, restore_ec);
  if (restore_ec && !ec) {
    ec = restore_ec;
    ok = false;
  }
  return ok;
}

bool Vl53l5cx::read_resolution_locked(Resolution &resolution, std::error_code &ec) {
  uint8_t zone_config[8];
  if (!dci_read_data(zone_config, DCI_ZONE_CONFIG, sizeof(zone_config), ec)) {
    return false;
  }
  const unsigned zones = static_cast<unsigned>(zone_config[0]) * zone_config[1];
  if (zones == 64) {
    resolution = Resolution::ZONES_8X8;
  } else if (zones == 16) {
    resolution = Resolution::ZONES_4X4;
  } else {
    logger_.error("Unexpected zone configuration {}x{}", zone_config[0], zone_config[1]);
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  resolution_ = resolution;
  return true;
}

Vl53l5cx::Resolution Vl53l5cx::get_resolution(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  Resolution resolution = Resolution::ZONES_4X4;
  read_resolution_locked(resolution, ec);
  return resolution;
}

bool Vl53l5cx::set_resolution(Resolution resolution, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (refuse_while_ranging("resolution", ec)) {
    return false;
  }
  uint8_t dss[16];
  uint8_t zone[8];
  if (!dci_read_data(dss, DCI_DSS_CONFIG, sizeof(dss), ec)) {
    return false;
  }
  switch (resolution) {
  case Resolution::ZONES_4X4:
    dss[0x04] = 64;
    dss[0x06] = 64;
    dss[0x09] = 4;
    break;
  case Resolution::ZONES_8X8:
    dss[0x04] = 16;
    dss[0x06] = 16;
    dss[0x09] = 1;
    break;
  default:
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  if (!dci_write_data(dss, DCI_DSS_CONFIG, sizeof(dss), ec) ||
      !dci_read_data(zone, DCI_ZONE_CONFIG, sizeof(zone), ec)) {
    return false;
  }
  if (resolution == Resolution::ZONES_4X4) {
    zone[0x00] = 4;
    zone[0x01] = 4;
    zone[0x04] = 8;
    zone[0x05] = 8;
  } else {
    zone[0x00] = 8;
    zone[0x01] = 8;
    zone[0x04] = 4;
    zone[0x05] = 4;
  }
  if (!dci_write_data(zone, DCI_ZONE_CONFIG, sizeof(zone), ec) ||
      !send_offset_data(resolution, ec) || !send_xtalk_data(resolution, ec)) {
    return false;
  }
  resolution_ = resolution;
  if (motion_enabled_) {
    motion_set_resolution(resolution);
    if (!write_motion_configuration(ec)) {
      return false;
    }
  }
  logger_.info("Resolution set to {}", resolution == Resolution::ZONES_8X8 ? "8x8" : "4x4");
  return true;
}

uint8_t Vl53l5cx::get_ranging_frequency_hz(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  uint8_t data[4];
  if (!dci_read_data(data, DCI_FREQ_HZ, sizeof(data), ec)) {
    return 0;
  }
  return data[0x01];
}

bool Vl53l5cx::set_ranging_frequency_hz(uint8_t frequency_hz, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (refuse_while_ranging("ranging frequency", ec)) {
    return false;
  }
  Resolution resolution = resolution_;
  if (!read_resolution_locked(resolution, ec)) {
    return false;
  }
  const uint8_t max_hz = resolution == Resolution::ZONES_8X8 ? 15 : 60;
  if (frequency_hz < 1 || frequency_hz > max_hz) {
    logger_.error("Ranging frequency {} Hz is outside 1..{} Hz for {}", frequency_hz, max_hz,
                  resolution == Resolution::ZONES_8X8 ? "8x8" : "4x4");
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  return dci_replace_data(DCI_FREQ_HZ, 4, &frequency_hz, 1, 0x01, ec);
}

uint32_t Vl53l5cx::get_integration_time_ms(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  uint8_t data[20];
  if (!dci_read_data(data, DCI_INT_TIME, sizeof(data), ec)) {
    return 0;
  }
  return load_u32(data) / 1000;
}

bool Vl53l5cx::set_integration_time_ms(uint32_t time_ms, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (refuse_while_ranging("integration time", ec)) {
    return false;
  }
  if (time_ms < 2 || time_ms > 1000) {
    logger_.error("Integration time {} ms is outside 2..1000 ms", time_ms);
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  uint8_t value[4];
  store_u32(value, time_ms * 1000);
  return dci_replace_data(DCI_INT_TIME, 20, value, sizeof(value), 0x00, ec);
}

uint8_t Vl53l5cx::get_sharpener_percent(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  uint8_t data[16];
  if (!dci_read_data(data, DCI_SHARPENER, sizeof(data), ec)) {
    return 0;
  }
  return static_cast<uint8_t>((static_cast<unsigned>(data[0xD]) * 100) / 255);
}

bool Vl53l5cx::set_sharpener_percent(uint8_t percent, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (refuse_while_ranging("sharpener", ec)) {
    return false;
  }
  if (percent >= 100) {
    logger_.error("Sharpener {} % is outside 0..99 %", percent);
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  const uint8_t sharpener = static_cast<uint8_t>((static_cast<unsigned>(percent) * 255) / 100);
  return dci_replace_data(DCI_SHARPENER, 16, &sharpener, 1, 0xD, ec);
}

Vl53l5cx::TargetOrder Vl53l5cx::get_target_order(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  uint8_t data[4];
  if (!dci_read_data(data, DCI_TARGET_ORDER, sizeof(data), ec)) {
    return TargetOrder::STRONGEST;
  }
  return data[0] == static_cast<uint8_t>(TargetOrder::CLOSEST) ? TargetOrder::CLOSEST
                                                               : TargetOrder::STRONGEST;
}

bool Vl53l5cx::set_target_order(TargetOrder order, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (refuse_while_ranging("target order", ec)) {
    return false;
  }
  if (order != TargetOrder::CLOSEST && order != TargetOrder::STRONGEST) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  const uint8_t value = static_cast<uint8_t>(order);
  return dci_replace_data(DCI_TARGET_ORDER, 4, &value, 1, 0x0, ec);
}

Vl53l5cx::RangingMode Vl53l5cx::get_ranging_mode(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  uint8_t data[8];
  if (!dci_read_data(data, DCI_RANGING_MODE, sizeof(data), ec)) {
    return RangingMode::AUTONOMOUS;
  }
  return data[0x01] == 0x1 ? RangingMode::CONTINUOUS : RangingMode::AUTONOMOUS;
}

bool Vl53l5cx::set_ranging_mode(RangingMode mode, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (refuse_while_ranging("ranging mode", ec)) {
    return false;
  }
  uint8_t data[8];
  if (!dci_read_data(data, DCI_RANGING_MODE, sizeof(data), ec)) {
    return false;
  }
  uint8_t single_range[4];
  switch (mode) {
  case RangingMode::CONTINUOUS:
    data[0x01] = 0x1;
    data[0x03] = 0x3;
    store_u32(single_range, 0x00);
    break;
  case RangingMode::AUTONOMOUS:
    data[0x01] = 0x3;
    data[0x03] = 0x2;
    store_u32(single_range, 0x01);
    break;
  default:
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  return dci_write_data(data, DCI_RANGING_MODE, sizeof(data), ec) &&
         dci_write_data(single_range, DCI_SINGLE_RANGE, sizeof(single_range), ec);
}

// ---------------------------------------------------------------------------
// Motion indicator (ULD plugin)
// ---------------------------------------------------------------------------

void Vl53l5cx::motion_set_resolution(Resolution resolution) {
  if (resolution == Resolution::ZONES_4X4) {
    for (int i = 0; i < 16; i++) {
      motion_config_.map_id[i] = static_cast<int8_t>(i);
    }
    std::memset(motion_config_.map_id + 16, -1, 48);
  } else {
    // 8x8 zones are aggregated 2x2 into the 16 motion aggregates
    for (int i = 0; i < 64; i++) {
      motion_config_.map_id[i] = static_cast<int8_t>(((i % 8) / 2) + (4 * (i / 16)));
    }
  }
}

bool Vl53l5cx::write_motion_configuration(std::error_code &ec) {
  uint8_t buffer[sizeof(MotionConfiguration)];
  std::memcpy(buffer, &motion_config_, sizeof(buffer));
  return dci_write_data(buffer, DCI_MOTION_DETECTOR_CFG, sizeof(buffer), ec);
}

bool Vl53l5cx::enable_motion_indicator(uint16_t distance_min_mm, uint16_t distance_max_mm,
                                       std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (refuse_while_ranging("motion indicator", ec)) {
    return false;
  }
  if (distance_min_mm < 400 || distance_max_mm > 4000 || distance_max_mm < distance_min_mm ||
      (distance_max_mm - distance_min_mm) > 1500) {
    logger_.error("Motion window {}..{} mm must lie in 400..4000 mm and span at most 1500 mm",
                  distance_min_mm, distance_max_mm);
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  // the ULD's vl53l5cx_motion_indicator_init() defaults ...
  std::memset(&motion_config_, 0, sizeof(motion_config_));
  motion_config_.ref_bin_offset = 13633;
  motion_config_.detection_threshold = 2883584;
  motion_config_.mem_update_mode = 6;
  motion_config_.mem_update_choice = 2;
  motion_config_.sum_span = 4;
  motion_config_.feature_length = 9;
  motion_config_.nb_of_aggregates = 16;
  motion_config_.nb_of_temporal_accumulations = 16;
  motion_config_.min_nb_for_global_detection = 1;
  motion_config_.global_indicator_format_1 = 8;
  motion_set_resolution(resolution_);
  // ... then vl53l5cx_motion_indicator_set_distance_motion()
  float tmp = ((static_cast<float>(distance_min_mm) / 37.5348f) - 4.0f) * 2048.5f;
  motion_config_.ref_bin_offset = static_cast<int32_t>(tmp);
  tmp =
      (((static_cast<float>(distance_max_mm - distance_min_mm) / 10.0f) + 30.02784f) / 15.01392f) +
      0.5f;
  motion_config_.feature_length = static_cast<uint8_t>(tmp);
  if (!write_motion_configuration(ec)) {
    return false;
  }
  motion_enabled_ = true;
  return true;
}

// ---------------------------------------------------------------------------
// Ranging
// ---------------------------------------------------------------------------

bool Vl53l5cx::start_ranging(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  Resolution resolution = resolution_;
  if (!read_resolution_locked(resolution, ec)) {
    return false;
  }
  const uint32_t zones = static_cast<uint32_t>(resolution);
  data_read_size_ = 0;
  stream_count_ = 255;

  // every output block is enabled (bit i of the first word enables output[i]):
  // the mandatory start / metadata / common data plus the nine result blocks
  uint32_t output_bh_enable[4] = {0x00000007 | 8 | 16 | 32 | 64 | 128 | 256 | 512 | 1024 | 2048,
                                  0x00000000, 0x00000000, 0xC0000000};
  uint32_t output[12] = {START_BH,      METADATA_BH,           COMMONDATA_BH,    AMBIENT_RATE_BH,
                         SPAD_COUNT_BH, NB_TARGET_DETECTED_BH, SIGNAL_RATE_BH,   RANGE_SIGMA_MM_BH,
                         DISTANCE_BH,   REFLECTANCE_BH,        TARGET_STATUS_BH, MOTION_DETECT_BH};
  // size the blocks for the resolution and add up the frame size
  uint32_t i = 0;
  for (; i < std::size(output); i++) {
    if (output[i] == 0 || (output_bh_enable[i / 32] & (1u << (i % 32))) == 0) {
      continue;
    }
    const uint32_t type = output[i] & 0xF;
    uint32_t size = (output[i] >> 4) & 0xFFF;
    const uint32_t idx = output[i] >> 16;
    if (type >= 0x1 && type < 0x0d) {
      if (idx >= 0x54d0 && idx < 0x54d0 + 960) {
        size = zones;
      } else {
        size = zones * TARGETS_PER_ZONE;
      }
      output[i] = (output[i] & 0xFFFF000F) | (size << 4);
      data_read_size_ += type * size;
    } else {
      data_read_size_ += size;
    }
    data_read_size_ += 4;
  }
  data_read_size_ += 20;
  if (data_read_size_ > TEMP_BUFFER_SIZE) {
    logger_.error("Frame size {} exceeds the buffer ({})", data_read_size_, TEMP_BUFFER_SIZE);
    ec = std::make_error_code(std::errc::no_buffer_space);
    return false;
  }
  uint8_t output_bytes[sizeof(output)];
  std::memcpy(output_bytes, output, sizeof(output));
  uint8_t header_config[8];
  store_u32(header_config, data_read_size_);
  store_u32(header_config + 4, i + 1);
  uint8_t enable_bytes[sizeof(output_bh_enable)];
  std::memcpy(enable_bytes, output_bh_enable, sizeof(enable_bytes));
  if (!dci_write_data(output_bytes, DCI_OUTPUT_LIST, sizeof(output_bytes), ec) ||
      !dci_write_data(header_config, DCI_OUTPUT_CONFIG, sizeof(header_config), ec) ||
      !dci_write_data(enable_bytes, DCI_OUTPUT_ENABLES, sizeof(enable_bytes), ec)) {
    return false;
  }
  // xshut bypass (interrupt mode), then the start command
  static constexpr uint8_t kStart[] = {0x00, 0x03, 0x00, 0x00};
  if (!write_byte(0x7fff, 0x00, ec) || !write_byte(0x09, 0x05, ec) ||
      !write_byte(0x7fff, 0x02, ec) ||
      !write_multi(UI_CMD_END - (sizeof(kStart) - 1), kStart, sizeof(kStart), ec) ||
      !poll_for_answer(4, 1, UI_CMD_STATUS, 0xff, 0x03, ec)) {
    logger_.error("Starting ranging failed");
    return false;
  }
  ranging_ = true;
  logger_.info("Ranging started ({} zones, {} byte frames)", zones, data_read_size_);
  return true;
}

bool Vl53l5cx::stop_ranging(std::error_code &ec) {
  using namespace std::chrono_literals;
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  uint8_t flag_bytes[4];
  if (!read_multi(0x2FFC, flag_bytes, sizeof(flag_bytes), ec)) {
    return false;
  }
  bool ok = true;
  if (load_u32(flag_bytes) != 0x4FF) {
    // the MCU is still ranging: provoke a stop and wait for it
    ok = write_byte(0x7fff, 0x00, ec) && write_byte(0x15, 0x16, ec) && write_byte(0x14, 0x01, ec);
    uint8_t tmp = 0;
    for (int attempt = 0; ok && (tmp & 0x80) == 0; ++attempt) {
      if (attempt > 500) { // 5 s
        logger_.error("The MCU did not stop ranging");
        ec = std::make_error_code(std::errc::timed_out);
        ok = false;
        break;
      }
      ok = read_byte(0x6, tmp, ec);
      std::this_thread::sleep_for(10ms);
    }
  }
  // undo the MCU stop and the xshut bypass whatever happened above
  std::error_code restore_ec;
  const bool restored = write_byte(0x7fff, 0x00, restore_ec) &&
                        write_byte(0x14, 0x00, restore_ec) && write_byte(0x15, 0x00, restore_ec) &&
                        write_byte(0x09, 0x04, restore_ec) && write_byte(0x7fff, 0x02, restore_ec);
  if (!restored && ok) {
    ec = restore_ec;
    ok = false;
  }
  ranging_ = false;
  return ok;
}

bool Vl53l5cx::is_data_ready(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  uint8_t header[4];
  if (!read_multi(0x0, header, sizeof(header), ec)) {
    return false;
  }
  // a new stream count with the "ranging / data valid" flags set
  if (header[0] != stream_count_ && header[0] != 255 && header[1] == 0x5 &&
      (header[2] & 0x5) == 0x5 && (header[3] & 0x10) == 0x10) {
    stream_count_ = header[0];
    return true;
  }
  return false;
}

bool Vl53l5cx::get_ranging_data(RangingData &data, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!ranging_ || data_read_size_ == 0) {
    logger_.error("Not ranging");
    ec = std::make_error_code(std::errc::operation_not_permitted);
    return false;
  }
  if (!read_multi(0x0, temp_buffer_.data(), data_read_size_, ec)) {
    return false;
  }
  stream_count_ = temp_buffer_[0];
  swap_buffer(temp_buffer_.data(), data_read_size_);
  data.resolution = resolution_;

  // walk the block headers (after the 16-byte frame header): type (4 bits),
  // size (12 bits), index (16 bits), then the block's data
  auto copy_block = [&](void *dst, size_t dst_size, size_t offset, size_t msize) {
    const size_t n = std::min(msize, dst_size);
    if (offset + n <= data_read_size_) {
      std::memcpy(dst, temp_buffer_.data() + offset, n);
    }
  };
  for (size_t i = 16; i + 4 <= data_read_size_; i += 4) {
    const uint32_t bh = load_u32(temp_buffer_.data() + i);
    const uint32_t type = bh & 0xF;
    const uint32_t size = (bh >> 4) & 0xFFF;
    const uint16_t idx = static_cast<uint16_t>(bh >> 16);
    const size_t msize = (type > 0x1 && type < 0xd) ? type * size : size;
    const size_t payload = i + 4;
    switch (idx) {
    case AMBIENT_RATE_IDX:
      copy_block(data.ambient_per_spad_kcps.data(), sizeof(data.ambient_per_spad_kcps), payload,
                 msize);
      break;
    case SPAD_COUNT_IDX:
      copy_block(data.nb_spads_enabled.data(), sizeof(data.nb_spads_enabled), payload, msize);
      break;
    case NB_TARGET_DETECTED_IDX:
      copy_block(data.nb_target_detected.data(), sizeof(data.nb_target_detected), payload, msize);
      break;
    case SIGNAL_RATE_IDX:
      copy_block(data.signal_per_spad_kcps.data(), sizeof(data.signal_per_spad_kcps), payload,
                 msize);
      break;
    case RANGE_SIGMA_MM_IDX:
      copy_block(data.range_sigma_mm.data(), sizeof(data.range_sigma_mm), payload, msize);
      break;
    case DISTANCE_IDX:
      copy_block(data.distance_mm.data(), sizeof(data.distance_mm), payload, msize);
      break;
    case REFLECTANCE_EST_PC_IDX:
      copy_block(data.reflectance_percent.data(), sizeof(data.reflectance_percent), payload, msize);
      break;
    case TARGET_STATUS_IDX:
      copy_block(data.target_status.data(), sizeof(data.target_status), payload, msize);
      break;
    case MOTION_DETEC_IDX:
      copy_block(&data.motion_indicator, sizeof(data.motion_indicator), payload, msize);
      break;
    default:
      break;
    }
    i += msize;
  }

  // convert the firmware's fixed-point formats
  for (size_t z = 0; z < MAX_ZONES; z++) {
    data.ambient_per_spad_kcps[z] /= 2048;
    data.distance_mm[z] = static_cast<int16_t>(std::max<int16_t>(data.distance_mm[z] / 4, 0));
    data.range_sigma_mm[z] /= 128;
    data.signal_per_spad_kcps[z] /= 2048;
    if (data.nb_target_detected[z] == 0) {
      data.target_status[z] = static_cast<uint8_t>(TargetStatus::NO_TARGET);
    }
  }
  auto &motion = data.motion_indicator.motion;
  std::transform(motion.begin(), motion.end(), motion.begin(),
                 [](uint32_t m) { return m / 65535; });
  return true;
}
