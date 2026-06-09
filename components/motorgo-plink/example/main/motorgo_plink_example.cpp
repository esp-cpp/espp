#include <array>
#include <cmath>
#include <numbers>
#include <system_error>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "logger.hpp"
#include "motorgo-plink.hpp"

namespace {
static auto logger = espp::Logger({
    .tag = "MotorGoPlink Example",
    .level = espp::Logger::Verbosity::INFO,
});

void log_board_info(espp::MotorGoPlink &board) {
  auto qwiic = board.qwiic_pins();
  auto hidden_i2c = board.hidden_i2c_pins();
  auto leds = board.led_pins();
  auto enc_bus = board.encoder_bus_pins();
  auto encoder_cs = board.encoder_cs_pins();
  auto servos = board.servo_pins();

  logger.info("Qwiic I2C: SDA={}, SCL={}", qwiic.sda, qwiic.scl);
  logger.info("Hidden I2C: SDA={}, SCL={}", hidden_i2c.sda, hidden_i2c.scl);
  logger.info("LEDs: user={}, status={}", leds.user, leds.status);
  logger.info("Encoder bus: SCLK={}, MISO={}, MOSI={}", enc_bus.sclk, enc_bus.miso, enc_bus.mosi);
  logger.info("Encoder CS: [{}, {}, {}, {}]", encoder_cs[0], encoder_cs[1], encoder_cs[2],
              encoder_cs[3]);
  logger.info("Servo pins: [{}, {}, {}, {}]", servos[0], servos[1], servos[2], servos[3]);
  for (size_t i = 0; i < espp::MotorGoPlink::num_motor_channels(); i++) {
    auto pins = board.motor_pins(i);
    logger.info("Motor {} pins: pwm_a={}, pwm_b={}", i + 1, pins.pwm_a, pins.pwm_b);
  }
}

void log_imu_data(espp::MotorGoPlink &board) {
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

#if CONFIG_MOTORGO_PLINK_EXAMPLE_ENABLE_ENCODER_POLLING
void log_encoder_angles(espp::MotorGoPlink &board) {
  std::array<float, espp::MotorGoPlink::num_motor_channels()> angles{};
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

  logger.info("Encoder angles [rad]: {:.3f}, {:.3f}, {:.3f}, {:.3f}", angles[0], angles[1],
              angles[2], angles[3]);
}
#endif

#if CONFIG_MOTORGO_PLINK_EXAMPLE_ENABLE_MOTOR_SWEEP
void update_motor_demo(espp::MotorGoPlink &board, float &phase) {
  constexpr float min_active_command = 0.35f;
  constexpr float max_command = 0.85f;
  constexpr float zero_window = 0.05f;
  constexpr float phase_step = 0.15f;
  constexpr float channel_offset = std::numbers::pi_v<float> / 2.0f;

  for (size_t i = 0; i < espp::MotorGoPlink::num_motor_channels(); i++) {
    float waveform = std::sin(phase + channel_offset * i);
    float speed = 0.0f;
    float magnitude = std::abs(waveform);
    if (magnitude > zero_window) {
      float normalized = (magnitude - zero_window) / (1.0f - zero_window);
      float commanded_magnitude =
          min_active_command + (max_command - min_active_command) * normalized;
      speed = std::copysign(commanded_magnitude, waveform);
    }
    board.set_motor_speed(i, speed);
  }

  phase = std::fmod(phase + phase_step, 2.0f * std::numbers::pi_v<float>);
}

void log_motor_outputs(espp::MotorGoPlink &board) {
  for (size_t i = 0; i < espp::MotorGoPlink::num_motor_channels(); i++) {
    auto driver = board.motor_driver(i);
    auto pins = board.motor_pins(i);
    if (!driver) {
      logger.warn("Motor {} driver is not initialized", i + 1);
      continue;
    }
    auto duty = driver->duty_cycle();
    auto raw = driver->raw_duty();
    auto max_raw_duty = driver->max_raw_duty();

    logger.info("Motor {} cmd={:.3f} | pwm_a gpio={} duty={:.1f}% raw={}/{} | pwm_b gpio={} "
                "duty={:.1f}% raw={}/{}",
                i + 1, board.motor_speed(i), static_cast<int>(pins.pwm_a), duty[0] * 100.0f, raw[0],
                max_raw_duty, static_cast<int>(pins.pwm_b), duty[1] * 100.0f, raw[1], max_raw_duty);
  }
}
#endif
} // namespace

extern "C" void app_main() {
  //! [motorgo-plink example]
  auto &board = espp::MotorGoPlink::get();

  if (!board.initialize_leds()) {
    logger.warn("Failed to initialize indicator LEDs");
  } else {
    board.start_led_breathing();
  }

#if CONFIG_MOTORGO_PLINK_EXAMPLE_ENABLE_MOTOR_SWEEP
  if (!board.initialize_motors()) {
    logger.error("Failed to initialize motor drivers");
  } else {
    board.stop_all_motors();
  }
#else
  logger.info("Motor drivers not initialized because motor sweep is disabled");
#endif

  if (!board.initialize_imu()) {
    logger.warn("Failed to initialize onboard IMU");
  } else {
    logger.info("IMU initialized");
  }

  log_board_info(board);

#if CONFIG_MOTORGO_PLINK_EXAMPLE_ENABLE_ENCODER_POLLING
  if (!board.initialize_encoders(false)) {
    logger.warn("Failed to initialize encoders");
  } else {
    logger.info("Encoder polling enabled");
  }
#else
  logger.info("Encoder polling disabled");
#endif

#if CONFIG_MOTORGO_PLINK_EXAMPLE_ENABLE_MOTOR_SWEEP
  logger.warn("Motor sweep enabled; using an aggressive {:.0f}% to {:.0f}% duty sweep after a "
              "{:.0f}% zero window. Ensure the board is connected to a safe test setup",
              35.0f, 85.0f, 5.0f);
#else
  logger.info("Motor sweep disabled; all motors remain stopped");
#endif

#if CONFIG_MOTORGO_PLINK_EXAMPLE_ENABLE_MOTOR_SWEEP
  float motor_phase = 0.0f;
#endif
  while (true) {
#if CONFIG_MOTORGO_PLINK_EXAMPLE_ENABLE_MOTOR_SWEEP
    update_motor_demo(board, motor_phase);
    log_motor_outputs(board);
#endif

#if CONFIG_MOTORGO_PLINK_EXAMPLE_ENABLE_ENCODER_POLLING
    log_encoder_angles(board);
#endif

    log_imu_data(board);

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(500ms);
  }
  //! [motorgo-plink example]
}
