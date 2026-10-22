#include "bno055.hpp"

#include <thread>

using namespace std::chrono_literals;

namespace espp {

namespace {

// Scale factors with UNIT_SEL = m/s^2, rad/s, rad, Celsius (datasheet section 3.6.4).
constexpr float kAccelLsbPerMps2 = 100.0f;
constexpr float kMagLsbPerUt = 16.0f;
constexpr float kGyroLsbPerRadps = 900.0f;
constexpr float kEulerLsbPerRad = 900.0f;
constexpr float kQuaternionLsbPerUnit = 16384.0f;

// UNIT_SEL: accel m/s^2 (bit0=0), gyro rad/s (bit1=1), Euler rad (bit2=1),
// temperature Celsius (bit4=0), Windows orientation convention (bit7=0).
constexpr uint8_t kUnitSelSi = 0b0000'0110;

constexpr uint8_t kSysTriggerReset = 0x20;
constexpr uint8_t kSysTriggerExternalCrystal = 0x80;
constexpr uint8_t kPowerModeNormal = 0x00;

int16_t to_i16(const uint8_t *bytes) {
  return static_cast<int16_t>(static_cast<uint16_t>(bytes[0]) |
                              (static_cast<uint16_t>(bytes[1]) << 8));
}

Bno055::Vector3 to_vector(const uint8_t *bytes, float lsb_per_unit) {
  return {to_i16(bytes) / lsb_per_unit, to_i16(bytes + 2) / lsb_per_unit,
          to_i16(bytes + 4) / lsb_per_unit};
}

} // namespace

Bno055::Bno055(const Config &config)
    : BasePeripheral({.address = config.device_address,
                      .probe = config.probe,
                      .write = config.write,
                      .write_then_read = config.write_then_read},
                     "Bno055", config.log_level)
    , mode_(config.mode)
    , use_external_crystal_(config.use_external_crystal) {
  if (config.auto_init) {
    std::error_code ec;
    if (!init(ec)) {
      logger_.error("Failed to initialize BNO055: {}", ec.message());
    }
  }
}

bool Bno055::init(std::error_code &ec) {
  initialized_ = false;

  // The sensor may still be booting (650 ms after power-on), so poll the chip ID.
  if (!wait_for_chip_id(ec)) {
    return false;
  }

  set_mode(OperatingMode::CONFIG, ec);
  if (ec) {
    return false;
  }

  write_u8_to_register(static_cast<uint8_t>(Register::SYS_TRIGGER), kSysTriggerReset, ec);
  // The sensor NACKs while resetting, so a write error here is expected.
  ec.clear();
  std::this_thread::sleep_for(650ms);
  if (!wait_for_chip_id(ec)) {
    return false;
  }

  write_u8_to_register(static_cast<uint8_t>(Register::PWR_MODE), kPowerModeNormal, ec);
  if (ec) {
    return false;
  }
  write_u8_to_register(static_cast<uint8_t>(Register::PAGE_ID), 0x00, ec);
  if (ec) {
    return false;
  }
  write_u8_to_register(static_cast<uint8_t>(Register::UNIT_SEL), kUnitSelSi, ec);
  if (ec) {
    return false;
  }
  write_u8_to_register(static_cast<uint8_t>(Register::SYS_TRIGGER),
                       use_external_crystal_ ? kSysTriggerExternalCrystal : 0x00, ec);
  if (ec) {
    return false;
  }
  std::this_thread::sleep_for(10ms);

  set_mode(mode_, ec);
  if (ec) {
    return false;
  }

  uint8_t self_test = read_u8_from_register(static_cast<uint8_t>(Register::ST_RESULT), ec);
  if (ec) {
    return false;
  }
  // ST_RESULT bits: 0 accel, 1 mag, 2 gyro, 3 MCU; 1 = passed.
  if ((self_test & 0x0F) != 0x0F) {
    logger_.warn("Self test incomplete: ST_RESULT=0x{:02x}", self_test);
  }

  initialized_ = true;
  logger_.info("Initialized in mode 0x{:02x}", static_cast<uint8_t>(mode_));
  return true;
}

bool Bno055::update(std::error_code &ec) {
  std::array<uint8_t, DATA_BLOCK_SIZE> raw{};
  read_many_from_register(static_cast<uint8_t>(Register::ACC_DATA_X_LSB), raw.data(), raw.size(),
                          ec);
  if (ec) {
    return false;
  }

  auto parsed = parse(raw);
  std::lock_guard<std::mutex> lock(data_mutex_);
  data_ = parsed;
  return true;
}

Bno055::Data Bno055::data() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return data_;
}

void Bno055::read_system_status(uint8_t &status, uint8_t &error, std::error_code &ec) {
  status = read_u8_from_register(static_cast<uint8_t>(Register::SYS_STATUS), ec);
  if (ec) {
    return;
  }
  error = read_u8_from_register(static_cast<uint8_t>(Register::SYS_ERR), ec);
}

void Bno055::set_mode(OperatingMode mode, std::error_code &ec) {
  write_u8_to_register(static_cast<uint8_t>(Register::OPR_MODE), static_cast<uint8_t>(mode), ec);
  // Datasheet table 3-6: 19 ms switching into CONFIG, 7 ms switching out of it.
  std::this_thread::sleep_for(mode == OperatingMode::CONFIG ? 25ms : 10ms);
}

bool Bno055::wait_for_chip_id(std::error_code &ec) {
  for (int attempt = 0; attempt < 20; ++attempt) {
    ec.clear();
    uint8_t id = read_u8_from_register(static_cast<uint8_t>(Register::CHIP_ID), ec);
    if (!ec && id == CHIP_ID) {
      return true;
    }
    std::this_thread::sleep_for(50ms);
  }
  if (!ec) {
    ec = std::make_error_code(std::errc::no_such_device);
  }
  logger_.error("No BNO055 chip ID (0x{:02x}) at address 0x{:02x}", CHIP_ID, base_config_.address);
  return false;
}

Bno055::Data Bno055::parse(const std::array<uint8_t, DATA_BLOCK_SIZE> &raw) {
  // Offsets are relative to ACC_DATA_X_LSB (0x08).
  const uint8_t *b = raw.data();
  Data data;
  data.accel_mps2 = to_vector(b + 0x00, kAccelLsbPerMps2);
  data.mag_ut = to_vector(b + 0x06, kMagLsbPerUt);
  data.gyro_radps = to_vector(b + 0x0C, kGyroLsbPerRadps);
  data.euler.heading = to_i16(b + 0x12) / kEulerLsbPerRad;
  data.euler.roll = to_i16(b + 0x14) / kEulerLsbPerRad;
  data.euler.pitch = to_i16(b + 0x16) / kEulerLsbPerRad;
  data.orientation.w = to_i16(b + 0x18) / kQuaternionLsbPerUnit;
  data.orientation.x = to_i16(b + 0x1A) / kQuaternionLsbPerUnit;
  data.orientation.y = to_i16(b + 0x1C) / kQuaternionLsbPerUnit;
  data.orientation.z = to_i16(b + 0x1E) / kQuaternionLsbPerUnit;
  data.linear_accel_mps2 = to_vector(b + 0x20, kAccelLsbPerMps2);
  data.gravity_mps2 = to_vector(b + 0x26, kAccelLsbPerMps2);
  data.temperature_c = static_cast<int8_t>(b[0x2C]);
  const uint8_t calib = b[0x2D];
  data.calibration = {.system = static_cast<uint8_t>((calib >> 6) & 0x03),
                      .gyro = static_cast<uint8_t>((calib >> 4) & 0x03),
                      .accel = static_cast<uint8_t>((calib >> 2) & 0x03),
                      .mag = static_cast<uint8_t>(calib & 0x03)};
  return data;
}

} // namespace espp
