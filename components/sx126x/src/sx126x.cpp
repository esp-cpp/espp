#include "sx126x.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <thread>

using namespace espp;

Sx126x::Sx126x(const Config &config)
    : BasePeripheral<uint8_t, false>(
          {.write = config.write, .write_then_read = config.write_then_read}, "Sx126x",
          config.log_level)
    , config_(config) {
  if (config_.auto_init) {
    std::error_code ec;
    initialize(ec);
    if (ec) {
      logger_.error("Failed to initialize radio: {}", ec.message());
    }
  }
}

bool Sx126x::initialize(std::error_code &ec) {
  logger_.info("Initializing SX126x");

  // hardware reset if we can, otherwise wake the chip with a standby command
  if (config_.reset) {
    config_.reset(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    config_.reset(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (!wait_on_busy(ec, std::chrono::milliseconds(100))) {
    logger_.error("Radio BUSY stuck high after reset");
    return false;
  }
  standby(ec);
  if (ec) {
    return false;
  }

  // verify communications: the LoRa sync word register has a known reset
  // value of 0x1424
  uint8_t sync[2] = {0, 0};
  read_registers(REG_SYNC_WORD_MSB, sync, 2, ec);
  if (ec) {
    return false;
  }
  if (sync[0] != 0x14 || sync[1] != 0x24) {
    logger_.error("Unexpected sync word reset value 0x{:02x}{:02x} - SPI comms failure?", sync[0],
                  sync[1]);
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }
  logger_.debug("SPI communications verified");

  // configure the regulator
  uint8_t reg_mode = config_.use_dcdc_regulator ? 0x01 : 0x00;
  cmd_write(CMD_SET_REGULATOR_MODE, &reg_mode, 1, ec);
  if (ec) {
    return false;
  }

  // configure DIO3 as TCXO control if requested. This must be done before
  // calibration, and requires re-calibration afterwards since the XOSC
  // failure flag will be set.
  if (config_.tcxo_voltage > 0.0f) {
    static constexpr float voltages[] = {1.6f, 1.7f, 1.8f, 2.2f, 2.4f, 2.7f, 3.0f, 3.3f};
    uint8_t code = 0;
    for (uint8_t i = 0; i < 8; i++) {
      if (config_.tcxo_voltage <= voltages[i] + 0.05f) {
        code = i;
        break;
      }
      code = 7;
    }
    uint32_t delay_steps = (config_.tcxo_delay.count() * 1000) / 15625; // 15.625 us steps
    uint8_t params[4] = {code, (uint8_t)((delay_steps >> 16) & 0xff),
                         (uint8_t)((delay_steps >> 8) & 0xff), (uint8_t)(delay_steps & 0xff)};
    cmd_write(CMD_SET_DIO3_AS_TCXO_CTRL, params, 4, ec);
    if (ec) {
      return false;
    }
  }

  calibrate_all(ec);
  if (ec) {
    return false;
  }

  // the XOSC-start failure error is expected after enabling TCXO control;
  // clear any errors from startup / calibration
  uint16_t errors = get_device_errors(ec);
  if (ec) {
    return false;
  }
  if (errors) {
    logger_.debug("Clearing device errors after calibration: 0x{:04x}", errors);
    clear_device_errors(ec);
    if (ec) {
      return false;
    }
  }

  if (config_.use_dio2_as_rf_switch) {
    uint8_t enable = 0x01;
    cmd_write(CMD_SET_DIO2_AS_RF_SWITCH, &enable, 1, ec);
    if (ec) {
      return false;
    }
  }

  // LoRa packet type
  uint8_t packet_type = 0x01;
  cmd_write(CMD_SET_PACKET_TYPE, &packet_type, 1, ec);
  if (ec) {
    return false;
  }

  // use the full 256-byte buffer for both TX and RX
  uint8_t base_addrs[2] = {0x00, 0x00};
  cmd_write(CMD_SET_BUFFER_BASE_ADDRESS, base_addrs, 2, ec);
  if (ec) {
    return false;
  }

  // fall back to standby (RC) after TX/RX completes
  uint8_t fallback = 0x20;
  cmd_write(CMD_SET_RX_TX_FALLBACK_MODE, &fallback, 1, ec);
  if (ec) {
    return false;
  }

  // route all interesting IRQs to DIO1
  set_dio_irq_params(
      IRQ_ALL,
      IRQ_TX_DONE | IRQ_RX_DONE | IRQ_CRC_ERR | IRQ_CAD_DONE | IRQ_CAD_DETECTED | IRQ_TIMEOUT, ec);
  if (ec) {
    return false;
  }

  initialized_ = true;

  if (!set_radio_config(config_.radio_config, ec)) {
    initialized_ = false;
    return false;
  }

  logger_.info("SX126x initialized");
  return true;
}

bool Sx126x::set_radio_config(const RadioConfig &config, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  if (config_.variant == Variant::LLCC68 && config.spreading_factor == SpreadingFactor::SF12) {
    logger_.error("LLCC68 does not support SF12");
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  receiving_ = false;
  standby(ec);
  if (ec) {
    return false;
  }
  config_.radio_config = config;
  set_frequency(config.frequency_hz, ec);
  if (ec) {
    return false;
  }
  set_pa_config(ec);
  if (ec) {
    return false;
  }
  set_tx_power(config.tx_power_dbm, ec);
  if (ec) {
    return false;
  }
  set_modulation_params(ec);
  if (ec) {
    return false;
  }
  set_packet_params(MAX_PAYLOAD_LENGTH, ec);
  if (ec) {
    return false;
  }
  set_sync_word(config.sync_word, ec);
  if (ec) {
    return false;
  }
  // boosted RX gain (register write is lost on sleep; we don't use warm-start
  // sleep so it only needs to be set here)
  write_radio_register(REG_RX_GAIN, config.rx_boosted_gain ? 0x96 : 0x94, ec);
  if (ec) {
    return false;
  }
  apply_workarounds(ec);
  if (ec) {
    return false;
  }
  return true;
}

void Sx126x::set_frequency(uint32_t frequency_hz, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  calibrate_image(frequency_hz, ec);
  if (ec) {
    return;
  }
  // freq_reg = freq_hz * 2^25 / 32e6
  uint32_t freq_reg = (uint32_t)(((uint64_t)frequency_hz * FREQ_DIV) / XTAL_FREQ);
  uint8_t params[4] = {(uint8_t)((freq_reg >> 24) & 0xff), (uint8_t)((freq_reg >> 16) & 0xff),
                       (uint8_t)((freq_reg >> 8) & 0xff), (uint8_t)(freq_reg & 0xff)};
  cmd_write(CMD_SET_RF_FREQUENCY, params, 4, ec);
  if (!ec) {
    config_.radio_config.frequency_hz = frequency_hz;
  }
}

void Sx126x::set_tx_power(int8_t power_dbm, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  if (config_.variant == Variant::SX1261) {
    power_dbm = std::clamp<int8_t>(power_dbm, -17, 15);
  } else {
    power_dbm = std::clamp<int8_t>(power_dbm, -9, 22);
  }
  uint8_t params[2] = {(uint8_t)power_dbm, (uint8_t)config_.radio_config.ramp_time};
  cmd_write(CMD_SET_TX_PARAMS, params, 2, ec);
  if (!ec) {
    config_.radio_config.tx_power_dbm = power_dbm;
  }
}

void Sx126x::set_sync_word(uint8_t sync_word, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // the 8-bit "public" sync word is spread across the two sync word
  // registers, with the low nibble of each fixed at 0x4
  uint8_t data[2] = {(uint8_t)((sync_word & 0xF0) | 0x04),
                     (uint8_t)(((sync_word & 0x0F) << 4) | 0x04)};
  write_registers(REG_SYNC_WORD_MSB, data, 2, ec);
  if (!ec) {
    config_.radio_config.sync_word = sync_word;
  }
}

void Sx126x::set_receive_callback(const receive_callback_fn &callback) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  config_.on_receive = callback;
}

void Sx126x::set_transmit_done_callback(const transmit_done_callback_fn &callback) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  config_.on_transmit_done = callback;
}

void Sx126x::set_cad_callback(const cad_callback_fn &callback) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  config_.on_cad_done = callback;
}

bool Sx126x::transmit(std::span<const uint8_t> data, std::chrono::milliseconds timeout,
                      std::error_code &ec) {
  bool was_receiving = receiving_;
  if (!start_transmit(data, ec)) {
    return false;
  }
  // Wait for completion. TX completion is signaled through tx_done_ /
  // tx_timeout_, which handle_dio1_interrupt() sets. That way this works both
  // when DIO1 is wired to an interrupt (the interrupt handler processes the
  // IRQ and sets the flag) and when it is not (we call handle_dio1_interrupt()
  // ourselves below) - without the two racing to read and clear the same
  // IRQ-status register.
  auto start = std::chrono::steady_clock::now();
  while (!tx_done_ && !tx_timeout_) {
    handle_dio1_interrupt(ec);
    if (ec) {
      return false;
    }
    if (tx_done_ || tx_timeout_) {
      break;
    }
    if (std::chrono::steady_clock::now() - start > timeout) {
      transmitting_ = false;
      // A transmission that never completes usually means the radio's
      // oscillator or PLL is not running - most often a wrong TCXO
      // configuration. Surface the device errors to make that diagnosable.
      std::error_code err_ec;
      uint16_t dev_errors = get_device_errors(err_ec);
      if (!err_ec && dev_errors) {
        logger_.error("TX timed out; radio device errors 0x{:04x} (0x20 = XOSC/TCXO start error, "
                      "0x40 = PLL lock error) - check the TCXO voltage",
                      dev_errors);
      } else {
        logger_.error("TX timed out (no radio device errors reported)");
      }
      standby(ec);
      ec = std::make_error_code(std::errc::timed_out);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (tx_timeout_) {
    logger_.error("TX timeout reported by radio");
    ec = std::make_error_code(std::errc::timed_out);
    return false;
  }
  if (was_receiving) {
    return start_receive(ec);
  }
  return true;
}

bool Sx126x::start_transmit(std::span<const uint8_t> data, std::error_code &ec) {
  if (data.empty() || data.size() > MAX_PAYLOAD_LENGTH) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  receiving_ = false;
  standby(ec);
  if (ec) {
    return false;
  }
  set_packet_params((uint8_t)data.size(), ec);
  if (ec) {
    return false;
  }
  write_tx_buffer(data, ec);
  if (ec) {
    return false;
  }
  clear_irq_status(IRQ_ALL, ec);
  if (ec) {
    return false;
  }
  tx_done_ = false;
  tx_timeout_ = false;
  // TX with no radio-side timeout (0x000000): the transmission runs to
  // completion and raises TX_DONE. The host-side timeout in transmit() is the
  // watchdog against a stuck radio. (A radio-side timeout derived from an
  // estimated time-on-air risks aborting a valid transmission if the estimate
  // is short, which surfaces as a spurious TX timeout.)
  uint8_t params[3] = {0x00, 0x00, 0x00};
  cmd_write(CMD_SET_TX, params, 3, ec);
  if (ec) {
    return false;
  }
  transmitting_ = true;
  return true;
}

bool Sx126x::start_receive(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  standby(ec);
  if (ec) {
    return false;
  }
  // restore max payload length for reception
  set_packet_params(MAX_PAYLOAD_LENGTH, ec);
  if (ec) {
    return false;
  }
  clear_irq_status(IRQ_ALL, ec);
  if (ec) {
    return false;
  }
  // continuous receive
  uint8_t params[3] = {0xFF, 0xFF, 0xFF};
  cmd_write(CMD_SET_RX, params, 3, ec);
  if (ec) {
    return false;
  }
  receiving_ = true;
  transmitting_ = false;
  return true;
}

bool Sx126x::start_cad(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  standby(ec);
  if (ec) {
    return false;
  }
  // CAD parameters recommended by AN1200.48 for the configured SF
  uint8_t sf = (uint8_t)config_.radio_config.spreading_factor;
  uint8_t det_peak = 0x15 + (sf >= 10 ? (sf - 9) : 0); // 21-24 depending on SF
  uint8_t params[7] = {
      0x03,                // CAD on 8 symbols
      det_peak,            // detection peak
      10,                  // detection minimum
      0x00,                // exit mode: back to standby-RC
      0x00,     0x00, 0x00 // CAD timeout (unused in exit mode 0)
  };
  cmd_write(CMD_SET_CAD_PARAMS, params, 7, ec);
  if (ec) {
    return false;
  }
  clear_irq_status(IRQ_ALL, ec);
  if (ec) {
    return false;
  }
  cmd_write(CMD_SET_CAD, nullptr, 0, ec);
  if (ec) {
    return false;
  }
  receiving_ = false;
  return true;
}

void Sx126x::standby(std::error_code &ec, StandbyMode mode) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  uint8_t param = (uint8_t)mode;
  cmd_write(CMD_SET_STANDBY, &param, 1, ec);
  if (!ec) {
    receiving_ = false;
    transmitting_ = false;
  }
}

void Sx126x::sleep(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // warm start (configuration retained), no RTC wakeup
  uint8_t param = 0x04;
  cmd_write(CMD_SET_SLEEP, &param, 1, ec);
  if (!ec) {
    receiving_ = false;
    transmitting_ = false;
  }
}

bool Sx126x::is_reception_in_progress(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  uint16_t irq = get_irq_status(ec);
  if (ec) {
    return false;
  }
  return (irq & (IRQ_PREAMBLE_DETECTED | IRQ_HEADER_VALID)) != 0;
}

float Sx126x::get_rssi_inst(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  uint8_t rssi_raw = 0;
  cmd_read(CMD_GET_RSSI_INST, nullptr, 0, &rssi_raw, 1, ec);
  if (ec) {
    return 0.0f;
  }
  return -(float)rssi_raw / 2.0f;
}

uint16_t Sx126x::handle_dio1_interrupt(std::error_code &ec) {
  RxPacket packet;
  bool got_packet = false;
  uint16_t irq = 0;
  {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    irq = get_irq_status(ec);
    if (ec) {
      return 0;
    }
    if (irq == IRQ_NONE) {
      return 0;
    }
    clear_irq_status(irq, ec);
    if (ec) {
      return irq;
    }
    logger_.debug("IRQ status: 0x{:04x}", irq);
    if ((irq & IRQ_RX_DONE) && !(irq & IRQ_CRC_ERR)) {
      got_packet = read_packet(packet, ec);
      // in continuous RX the radio stays in RX; nothing to restart
    } else if (irq & IRQ_CRC_ERR) {
      logger_.warn("Dropping received packet with CRC error");
    }
    bool was_transmitting = transmitting_;
    if (irq & IRQ_TX_DONE) {
      transmitting_ = false;
      tx_done_ = true;
      // fallback mode has put us in standby; resume receiving if we were
      if (receiving_) {
        start_receive(ec);
      }
    }
    if (irq & IRQ_TIMEOUT) {
      transmitting_ = false;
      // only a TX timeout concerns a waiting transmit(); RX timeouts (not used
      // in continuous-RX mode) are ignored
      if (was_transmitting) {
        tx_timeout_ = true;
      }
    }
  }
  // invoke callbacks outside the lock
  if (got_packet && config_.on_receive) {
    config_.on_receive(packet);
  }
  if ((irq & IRQ_TX_DONE) && config_.on_transmit_done) {
    config_.on_transmit_done();
  }
  if ((irq & IRQ_CAD_DONE) && config_.on_cad_done) {
    config_.on_cad_done((irq & IRQ_CAD_DETECTED) != 0);
  }
  return irq;
}

std::chrono::milliseconds Sx126x::time_on_air(size_t payload_length) const {
  // Semtech AN1200.13 LoRa time-on-air calculation
  const auto &rc = config_.radio_config;
  int sf = (int)rc.spreading_factor;
  float bw = bandwidth_hz();
  float t_sym_ms = (float)(1 << sf) / bw * 1000.0f;
  int de = low_data_rate_optimize() ? 1 : 0;
  int cr = (int)rc.coding_rate; // 1..4
  int crc = rc.crc_enabled ? 1 : 0;
  int ih = 0; // explicit header
  float num = 8.0f * payload_length - 4.0f * sf + 28.0f + 16.0f * crc - 20.0f * ih;
  float den = 4.0f * (sf - 2 * de);
  float n_payload = 8.0f + std::max(std::ceil(num / den) * (cr + 4), 0.0f);
  float t_preamble_ms = (rc.preamble_length + 4.25f) * t_sym_ms;
  float t_payload_ms = n_payload * t_sym_ms;
  return std::chrono::milliseconds((int64_t)std::ceil(t_preamble_ms + t_payload_ms));
}

uint8_t Sx126x::read_radio_register(uint16_t address, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  uint8_t value = 0;
  read_registers(address, &value, 1, ec);
  return value;
}

void Sx126x::write_radio_register(uint16_t address, uint8_t value, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  write_registers(address, &value, 1, ec);
}

/////////////////////////////////////////////////////////////////////////////
// Protected / low-level
/////////////////////////////////////////////////////////////////////////////

bool Sx126x::wait_on_busy(std::error_code &ec, std::chrono::milliseconds timeout) {
  if (!config_.is_busy) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  auto start = std::chrono::steady_clock::now();
  while (config_.is_busy()) {
    if (std::chrono::steady_clock::now() - start > timeout) {
      logger_.error("Timed out waiting for BUSY to deassert");
      ec = std::make_error_code(std::errc::timed_out);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  return true;
}

void Sx126x::cmd_write(uint8_t opcode, const uint8_t *params, size_t length, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  if (!wait_on_busy(ec, std::chrono::milliseconds(100))) {
    return;
  }
  uint8_t buffer[1 + 16];
  if (length + 1 > sizeof(buffer)) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return;
  }
  buffer[0] = opcode;
  if (params && length) {
    std::memcpy(&buffer[1], params, length);
  }
  write_many(buffer, length + 1, ec);
}

void Sx126x::cmd_read(uint8_t opcode, const uint8_t *params, size_t params_length,
                      uint8_t *response, size_t response_length, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  if (!wait_on_busy(ec, std::chrono::milliseconds(100))) {
    return;
  }
  // read commands return an RFU/status byte after the opcode (+ params) which
  // must be clocked out before the data
  uint8_t buffer[1 + 4];
  if (params_length + 2 > sizeof(buffer)) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return;
  }
  buffer[0] = opcode;
  if (params && params_length) {
    std::memcpy(&buffer[1], params, params_length);
  }
  buffer[1 + params_length] = 0x00; // NOP to clock out the status byte
  write_then_read(buffer, params_length + 2, response, response_length, ec);
}

void Sx126x::write_registers(uint16_t address, const uint8_t *data, size_t length,
                             std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  if (!wait_on_busy(ec, std::chrono::milliseconds(100))) {
    return;
  }
  std::vector<uint8_t> buffer(3 + length);
  buffer[0] = CMD_WRITE_REGISTER;
  buffer[1] = (address >> 8) & 0xff;
  buffer[2] = address & 0xff;
  std::memcpy(&buffer[3], data, length);
  write_many(buffer, ec);
}

void Sx126x::read_registers(uint16_t address, uint8_t *data, size_t length, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  if (!wait_on_busy(ec, std::chrono::milliseconds(100))) {
    return;
  }
  uint8_t cmd[4] = {CMD_READ_REGISTER, (uint8_t)((address >> 8) & 0xff), (uint8_t)(address & 0xff),
                    0x00};
  write_then_read(cmd, 4, data, length, ec);
}

void Sx126x::write_tx_buffer(std::span<const uint8_t> data, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  if (!wait_on_busy(ec, std::chrono::milliseconds(100))) {
    return;
  }
  std::vector<uint8_t> buffer(2 + data.size());
  buffer[0] = CMD_WRITE_BUFFER;
  buffer[1] = 0x00; // offset
  std::memcpy(&buffer[2], data.data(), data.size());
  write_many(buffer, ec);
}

void Sx126x::set_packet_params(uint8_t payload_length, std::error_code &ec) {
  const auto &rc = config_.radio_config;
  uint8_t params[6] = {
      (uint8_t)((rc.preamble_length >> 8) & 0xff),
      (uint8_t)(rc.preamble_length & 0xff),
      0x00, // explicit (variable length) header
      payload_length,
      (uint8_t)(rc.crc_enabled ? 0x01 : 0x00),
      (uint8_t)(rc.invert_iq ? 0x01 : 0x00),
  };
  cmd_write(CMD_SET_PACKET_PARAMS, params, 6, ec);
  if (ec) {
    return;
  }
  // datasheet workaround: the IQ polarity register must be adjusted to match
  // the configured IQ inversion for optimal RX sensitivity
  uint8_t iq_polarity = read_radio_register(REG_IQ_POLARITY, ec);
  if (ec) {
    return;
  }
  if (rc.invert_iq) {
    iq_polarity &= ~0x04;
  } else {
    iq_polarity |= 0x04;
  }
  write_radio_register(REG_IQ_POLARITY, iq_polarity, ec);
}

void Sx126x::set_modulation_params(std::error_code &ec) {
  const auto &rc = config_.radio_config;
  uint8_t params[4] = {
      (uint8_t)rc.spreading_factor,
      (uint8_t)rc.bandwidth,
      (uint8_t)rc.coding_rate,
      (uint8_t)(low_data_rate_optimize() ? 0x01 : 0x00),
  };
  cmd_write(CMD_SET_MODULATION_PARAMS, params, 4, ec);
}

void Sx126x::set_pa_config(std::error_code &ec) {
  uint8_t params[4];
  if (config_.variant == Variant::SX1261) {
    // optimal +14/+15 dBm configuration
    params[0] = 0x04;
    params[1] = 0x00;
    params[2] = 0x01;
    params[3] = 0x01;
  } else {
    // SX1262 / LLCC68 optimal +22 dBm configuration
    params[0] = 0x04;
    params[1] = 0x07;
    params[2] = 0x00;
    params[3] = 0x01;
  }
  cmd_write(CMD_SET_PA_CONFIG, params, 4, ec);
}

void Sx126x::set_dio_irq_params(uint16_t irq_mask, uint16_t dio1_mask, std::error_code &ec) {
  uint8_t params[8] = {
      (uint8_t)((irq_mask >> 8) & 0xff),
      (uint8_t)(irq_mask & 0xff),
      (uint8_t)((dio1_mask >> 8) & 0xff),
      (uint8_t)(dio1_mask & 0xff),
      0x00,
      0x00, // DIO2 (may be RF switch)
      0x00,
      0x00, // DIO3 (may be TCXO)
  };
  cmd_write(CMD_SET_DIO_IRQ_PARAMS, params, 8, ec);
}

uint16_t Sx126x::get_irq_status(std::error_code &ec) {
  uint8_t status[2] = {0, 0};
  cmd_read(CMD_GET_IRQ_STATUS, nullptr, 0, status, 2, ec);
  if (ec) {
    return 0;
  }
  return ((uint16_t)status[0] << 8) | status[1];
}

void Sx126x::clear_irq_status(uint16_t mask, std::error_code &ec) {
  uint8_t params[2] = {(uint8_t)((mask >> 8) & 0xff), (uint8_t)(mask & 0xff)};
  cmd_write(CMD_CLEAR_IRQ_STATUS, params, 2, ec);
}

uint16_t Sx126x::get_device_errors(std::error_code &ec) {
  uint8_t errors[2] = {0, 0};
  cmd_read(CMD_GET_DEVICE_ERRORS, nullptr, 0, errors, 2, ec);
  if (ec) {
    return 0;
  }
  return ((uint16_t)errors[0] << 8) | errors[1];
}

void Sx126x::clear_device_errors(std::error_code &ec) {
  uint8_t params[2] = {0x00, 0x00};
  cmd_write(CMD_CLEAR_DEVICE_ERRORS, params, 2, ec);
}

void Sx126x::calibrate_all(std::error_code &ec) {
  uint8_t param = 0x7F; // all blocks
  cmd_write(CMD_CALIBRATE, &param, 1, ec);
  if (ec) {
    return;
  }
  // full calibration takes up to ~3.5 ms; BUSY is high for the duration
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  wait_on_busy(ec, std::chrono::milliseconds(100));
}

void Sx126x::calibrate_image(uint32_t frequency_hz, std::error_code &ec) {
  uint8_t params[2];
  uint32_t mhz = frequency_hz / 1000000;
  if (mhz >= 902) {
    params[0] = 0xE1;
    params[1] = 0xE9;
  } else if (mhz >= 863) {
    params[0] = 0xD7;
    params[1] = 0xDB;
  } else if (mhz >= 779) {
    params[0] = 0xC1;
    params[1] = 0xC5;
  } else if (mhz >= 470) {
    params[0] = 0x75;
    params[1] = 0x81;
  } else {
    params[0] = 0x6B;
    params[1] = 0x6F;
  }
  cmd_write(CMD_CALIBRATE_IMAGE, params, 2, ec);
  if (ec) {
    return;
  }
  wait_on_busy(ec, std::chrono::milliseconds(100));
}

void Sx126x::apply_workarounds(std::error_code &ec) {
  // datasheet 15.1: modulation quality with 500 kHz bandwidth
  uint8_t tx_mod = read_radio_register(REG_TX_MODULATION, ec);
  if (ec) {
    return;
  }
  if (config_.radio_config.bandwidth == Bandwidth::BW_500_KHZ) {
    tx_mod &= ~0x04;
  } else {
    tx_mod |= 0x04;
  }
  write_radio_register(REG_TX_MODULATION, tx_mod, ec);
  if (ec) {
    return;
  }
  // datasheet 15.2: better resistance to antenna mismatch (SX1262 PA)
  if (config_.variant != Variant::SX1261) {
    uint8_t clamp = read_radio_register(REG_TX_CLAMP_CONFIG, ec);
    if (ec) {
      return;
    }
    clamp |= 0x1E;
    write_radio_register(REG_TX_CLAMP_CONFIG, clamp, ec);
  }
}

bool Sx126x::read_packet(RxPacket &packet, std::error_code &ec) {
  uint8_t buffer_status[2] = {0, 0};
  cmd_read(CMD_GET_RX_BUFFER_STATUS, nullptr, 0, buffer_status, 2, ec);
  if (ec) {
    return false;
  }
  uint8_t length = buffer_status[0];
  uint8_t offset = buffer_status[1];
  if (length == 0) {
    return false;
  }
  packet.data.resize(length);
  uint8_t params[1] = {offset};
  cmd_read(CMD_READ_BUFFER, params, 1, packet.data.data(), length, ec);
  if (ec) {
    return false;
  }
  packet.status = get_packet_status(ec);
  return !ec;
}

Sx126x::PacketStatus Sx126x::get_packet_status(std::error_code &ec) {
  PacketStatus status;
  uint8_t raw[3] = {0, 0, 0};
  cmd_read(CMD_GET_PACKET_STATUS, nullptr, 0, raw, 3, ec);
  if (ec) {
    return status;
  }
  status.rssi = -(float)raw[0] / 2.0f;
  status.snr = (float)(int8_t)raw[1] / 4.0f;
  status.signal_rssi = -(float)raw[2] / 2.0f;
  return status;
}

bool Sx126x::low_data_rate_optimize() const {
  // enable LDRO when the symbol time exceeds 16.38 ms (per datasheet
  // recommendation): SF11/SF12 @ 125 kHz, SF12 @ 250 kHz
  int sf = (int)config_.radio_config.spreading_factor;
  float t_sym_ms = (float)(1 << sf) / bandwidth_hz() * 1000.0f;
  return t_sym_ms > 16.38f;
}

float Sx126x::bandwidth_hz() const {
  switch (config_.radio_config.bandwidth) {
  case Bandwidth::BW_7_8_KHZ:
    return 7810.0f;
  case Bandwidth::BW_10_4_KHZ:
    return 10420.0f;
  case Bandwidth::BW_15_6_KHZ:
    return 15630.0f;
  case Bandwidth::BW_20_8_KHZ:
    return 20830.0f;
  case Bandwidth::BW_31_25_KHZ:
    return 31250.0f;
  case Bandwidth::BW_41_7_KHZ:
    return 41670.0f;
  case Bandwidth::BW_62_5_KHZ:
    return 62500.0f;
  case Bandwidth::BW_125_KHZ:
    return 125000.0f;
  case Bandwidth::BW_250_KHZ:
    return 250000.0f;
  case Bandwidth::BW_500_KHZ:
    return 500000.0f;
  }
  return 125000.0f;
}
