#include <array>
#include <system_error>
#include <thread>

#include <esp_timer.h>

#include "logger.hpp"
#include "motorgo-axis.hpp"

namespace {
static auto logger = espp::Logger({
    .tag = "MotorGoAxis Example",
    .level = espp::Logger::Verbosity::INFO,
});

void log_board_info(espp::MotorGoAxis &board) {
  auto qwiic = board.qwiic_pins();
  auto hidden_i2c = board.hidden_i2c_pins();
  auto leds = board.led_pins();
  auto enc_bus = board.encoder_bus_pins();
  auto encoder_cs = board.encoder_cs_pins();

  logger.info("Qwiic I2C: SDA={}, SCL={}", qwiic.sda, qwiic.scl);
  logger.info("Hidden I2C: SDA={}, SCL={}", hidden_i2c.sda, hidden_i2c.scl);
  logger.info("LEDs: user={}, status={}", leds.user, leds.status);
  logger.info("Encoder bus: SCLK={}, MISO={}, MOSI={}", enc_bus.sclk, enc_bus.miso, enc_bus.mosi);
  logger.info("Encoder CS: [{}, {}]", encoder_cs[0], encoder_cs[1]);
  for (size_t i = 0; i < espp::MotorGoAxis::num_motor_channels(); i++) {
    auto pins = board.motor_pins(i);
    logger.info("Motor {} pins: uh={}, ul={}, vh={}, vl={}, wh={}, wl={}", i + 1, pins.u_high,
                pins.u_low, pins.v_high, pins.v_low, pins.w_high, pins.w_low);
  }
}

void log_imu_data(espp::MotorGoAxis &board) {
  auto imu = board.imu();
  if (!imu) {
    logger.warn("IMU is not initialized");
    return;
  }

  static int64_t last_update_us = esp_timer_get_time();
  int64_t now_us = esp_timer_get_time();
  float dt = (now_us - last_update_us) / 1'000'000.0f;
  last_update_us = now_us;

  std::error_code ec;
  if (!imu->update(dt, ec)) {
    logger.warn("Failed to update IMU: {}", ec.message());
    return;
  }

  auto accel = imu->get_accelerometer();
  auto gyro = imu->get_gyroscope();
  auto temp = imu->get_temperature();
  logger.info("IMU accel [g]: {:.3f}, {:.3f}, {:.3f} | gyro [dps]: {:.3f}, {:.3f}, {:.3f} | "
              "temp [C]: {:.1f}",
              accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, temp);
}

#if CONFIG_MOTORGO_AXIS_EXAMPLE_ENABLE_ENCODER_POLLING
void log_encoder_angles(espp::MotorGoAxis &board) {
  std::array<float, espp::MotorGoAxis::num_motor_channels()> angles{};
  for (size_t i = 0; i < angles.size(); i++) {
    auto encoder = board.encoder(i);
    if (!encoder) {
      logger.warn("Encoder {} is not initialized", i + 1);
      return;
    }
    std::error_code ec;
    encoder->update(ec);
    if (ec) {
      logger.warn("Failed to update encoder {}: {}", i + 1, ec.message());
      return;
    }
    angles[i] = encoder->get_radians();
  }

  logger.info("Encoder angles [rad]: {:.3f}, {:.3f}", angles[0], angles[1]);
}
#endif
} // namespace

extern "C" void app_main() {
  //! [motorgo-axis example]
  auto &board = espp::MotorGoAxis::get();

  if (!board.initialize_leds()) {
    logger.warn("Failed to initialize indicator LEDs");
  } else {
    board.start_led_breathing();
  }

  if (!board.initialize_motors()) {
    logger.warn("Failed to initialize BLDC motor drivers");
  } else {
    logger.info("Motor drivers initialized and left disabled");
  }

  if (!board.initialize_imu()) {
    logger.warn("Failed to initialize onboard IMU");
  } else {
    logger.info("IMU initialized");
  }

  log_board_info(board);

#if CONFIG_MOTORGO_AXIS_EXAMPLE_ENABLE_ENCODER_POLLING
  if (!board.initialize_encoders(false)) {
    logger.warn("Failed to initialize encoders");
  } else {
    logger.info("Encoder polling enabled");
  }
#else
  logger.info("Encoder polling disabled");
#endif

  while (true) {
#if CONFIG_MOTORGO_AXIS_EXAMPLE_ENABLE_ENCODER_POLLING
    log_encoder_angles(board);
#endif

    log_imu_data(board);

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(500ms);
  }
  //! [motorgo-axis example]
}
