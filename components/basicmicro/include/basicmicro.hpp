#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include "base_component.hpp"
#include "detail/basicmicro_core.hpp"

namespace espp {

/**
 * @brief Driver for Basicmicro (MCP236 / MCP266 and RoboClaw-family) brushed
 *        DC motor controllers speaking the PACKET SERIAL protocol, typically
 *        over UART.
 *
 * The component is transport-agnostic: it performs no I/O itself and instead
 * calls the user-provided write / read functions for each transaction, so it
 * works over a UART driver, USB CDC, RS-232 adapter, etc. Wire-format logic
 * (CRC16, packet building, reply validation, big-endian codecs) lives in
 * `include/detail/basicmicro_core.hpp`, a host-buildable core with zero ESP
 * dependencies.
 *
 * Protocol summary (MCP Series User Manual section 2.2):
 *  - Write commands send [Address, Command, Data..., CRC16] and the controller
 *    replies with a single 0xFF ACK byte only when the packet was valid.
 *  - Read commands send [Address, Command] (no CRC) and the controller replies
 *    with the data followed by a CRC16 seeded with the sent address + command.
 *  - Error recovery: a >=10 ms gap between bytes makes the controller discard
 *    any partial packet, so the configured receive timeout (>=10 ms, default
 *    20 ms) doubles as the recovery mechanism — after a timed-out transaction
 *    the controller's packet buffer has already cleared itself and the next
 *    packet starts fresh.
 *
 * All methods report errors via a std::error_code out-parameter (no
 * exceptions) and return true on success.
 *
 * @note Thread safety: every public method runs one complete transaction
 *       (write the request, then read the ACK / reply) while holding an
 *       internal mutex, so concurrent calls from multiple tasks serialize
 *       cleanly and replies cannot interleave. The user-provided read / write
 *       functions ARE called with that mutex held — this is intentional, since
 *       the transaction is precisely the I/O — so they must not call back into
 *       this component.
 *
 * \section basicmicro_ex1 Basicmicro Example
 * \snippet basicmicro_example.cpp basicmicro example
 */
class Basicmicro : public BaseComponent {
public:
  /// Command bytes (verified against the MCP Series User Manual).
  using Command = detail::BasicmicroCommand;
  /// Status bit masks returned by read_status() (manual command 90).
  using Status = detail::BasicmicroStatus;

  /// Function used to transmit a complete packet to the controller.
  /// Should return true when all bytes were written.
  typedef std::function<bool(std::span<const uint8_t> data)> write_fn;

  /// Function used to receive reply bytes from the controller. Should block
  /// until at least one byte is available or the timeout expires, and return
  /// the number of bytes actually read into the span (0 on timeout).
  typedef std::function<size_t(std::span<uint8_t> data, std::chrono::milliseconds timeout)> read_fn;

  /// Configuration for the Basicmicro driver.
  struct Config {
    uint8_t address{0x80}; /**< Packet serial address of the controller (0x80 - 0x87). */
    write_fn write;        /**< Function to write bytes to the controller. */
    read_fn read;          /**< Function to read bytes from the controller. */
    std::chrono::milliseconds timeout{
        20}; /**< Total receive timeout per transaction. Must be >= 10 ms: the controller
                discards a partial packet after a 10 ms inter-byte gap (manual section
                2.2.4), so waiting at least that long guarantees its packet buffer has
                cleared before the next transaction. */
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; /**< Logger verbosity. */
  };

  /**
   * @brief Create a Basicmicro driver.
   * @param config Configuration parameters.
   * @note The documented contracts are enforced here: a timeout below the
   *       protocol's 10 ms packet-clear window is clamped up to 10 ms, and an
   *       address outside 0x80-0x87 is clamped into the valid range (both with
   *       a warning) rather than silently violating the wire protocol.
   */
  explicit Basicmicro(const Config &config)
      : BaseComponent("Basicmicro", config.log_level)
      , config_(config) {
    if (config_.timeout < std::chrono::milliseconds(detail::kBasicmicroPacketTimeoutMs)) {
      logger_.warn("timeout {} ms is below the protocol's {} ms packet-clear window; clamping",
                   config_.timeout.count(), detail::kBasicmicroPacketTimeoutMs);
      config_.timeout = std::chrono::milliseconds(detail::kBasicmicroPacketTimeoutMs);
    }
    if (config_.address < detail::kBasicmicroMinAddress ||
        config_.address > detail::kBasicmicroMaxAddress) {
      const uint8_t clamped =
          std::clamp(config_.address, detail::kBasicmicroMinAddress, detail::kBasicmicroMaxAddress);
      logger_.warn("address {:#04x} is outside the valid packet-serial range [{:#04x}, {:#04x}]; "
                   "clamping to {:#04x}",
                   config_.address, detail::kBasicmicroMinAddress, detail::kBasicmicroMaxAddress,
                   clamped);
      config_.address = clamped;
    }
  }

  // ------------------------- duty-cycle drive ------------------------------

  /**
   * @brief Drive motor 1 with a signed duty cycle (command 32).
   * @param duty Signed duty, -32767 to +32767 (= -100% to +100%).
   * @param ec Set on failure.
   * @return True on success.
   */
  bool drive_m1_duty(int16_t duty, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_i16_be(payload, duty);
    return write_command(Command::DriveM1SignedDuty, payload, ec);
  }

  /**
   * @brief Drive motor 2 with a signed duty cycle (command 33).
   * @param duty Signed duty, -32767 to +32767 (= -100% to +100%).
   * @param ec Set on failure.
   * @return True on success.
   */
  bool drive_m2_duty(int16_t duty, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_i16_be(payload, duty);
    return write_command(Command::DriveM2SignedDuty, payload, ec);
  }

  /**
   * @brief Drive both motors with signed duty cycles (command 34).
   * @param duty_m1 Signed duty for motor 1, -32767 to +32767.
   * @param duty_m2 Signed duty for motor 2, -32767 to +32767.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool drive_duty(int16_t duty_m1, int16_t duty_m2, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_i16_be(payload, duty_m1);
    detail::append_i16_be(payload, duty_m2);
    return write_command(Command::DriveM1M2SignedDuty, payload, ec);
  }

  // ------------------------ closed-loop speed drive ------------------------

  /**
   * @brief Drive motor 1 at a signed speed in quadrature pulses per second
   *        (command 35). Requires an encoder and tuned velocity PID.
   * @param qpps Signed speed in quad pulses per second.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool drive_m1_speed(int32_t qpps, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_i32_be(payload, qpps);
    return write_command(Command::DriveM1SignedSpeed, payload, ec);
  }

  /**
   * @brief Drive motor 2 at a signed speed in quadrature pulses per second
   *        (command 36). Requires an encoder and tuned velocity PID.
   * @param qpps Signed speed in quad pulses per second.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool drive_m2_speed(int32_t qpps, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_i32_be(payload, qpps);
    return write_command(Command::DriveM2SignedSpeed, payload, ec);
  }

  /**
   * @brief Drive both motors at signed speeds in quadrature pulses per second
   *        (command 37).
   * @param qpps_m1 Signed speed for motor 1 in quad pulses per second.
   * @param qpps_m2 Signed speed for motor 2 in quad pulses per second.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool drive_speed(int32_t qpps_m1, int32_t qpps_m2, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_i32_be(payload, qpps_m1);
    detail::append_i32_be(payload, qpps_m2);
    return write_command(Command::DriveM1M2SignedSpeed, payload, ec);
  }

  /**
   * @brief Drive motor 1 at a signed speed with an acceleration ramp
   *        (command 38).
   * @param accel Acceleration in qpps per second (unsigned).
   * @param qpps Signed target speed in quad pulses per second.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool drive_m1_speed_accel(uint32_t accel, int32_t qpps, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_u32_be(payload, accel);
    detail::append_i32_be(payload, qpps);
    return write_command(Command::DriveM1SignedSpeedAccel, payload, ec);
  }

  /**
   * @brief Drive motor 2 at a signed speed with an acceleration ramp
   *        (command 39).
   * @param accel Acceleration in qpps per second (unsigned).
   * @param qpps Signed target speed in quad pulses per second.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool drive_m2_speed_accel(uint32_t accel, int32_t qpps, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_u32_be(payload, accel);
    detail::append_i32_be(payload, qpps);
    return write_command(Command::DriveM2SignedSpeedAccel, payload, ec);
  }

  /**
   * @brief Drive both motors at signed speeds with a shared acceleration ramp
   *        (command 40).
   * @param accel Acceleration in qpps per second (unsigned, applies to both).
   * @param qpps_m1 Signed target speed for motor 1.
   * @param qpps_m2 Signed target speed for motor 2.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool drive_speed_accel(uint32_t accel, int32_t qpps_m1, int32_t qpps_m2, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_u32_be(payload, accel);
    detail::append_i32_be(payload, qpps_m1);
    detail::append_i32_be(payload, qpps_m2);
    return write_command(Command::DriveM1M2SignedSpeedAccel, payload, ec);
  }

  // -------------------------- buffered motion ------------------------------

  /**
   * @brief Buffered drive of motor 1 with signed speed and distance
   *        (command 41).
   * @param qpps Signed speed in quad pulses per second.
   * @param distance Distance in quad pulses (unsigned).
   * @param immediate If true, stop the currently-executing command, flush the
   *        buffer and run this command now; if false, queue it (up to 64
   *        commands per motor buffer).
   * @param ec Set on failure.
   * @return True on success.
   */
  bool buffered_drive_m1_speed_distance(int32_t qpps, uint32_t distance, bool immediate,
                                        std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_i32_be(payload, qpps);
    detail::append_u32_be(payload, distance);
    detail::append_u8(payload, immediate ? 1 : 0);
    return write_command(Command::BufferedM1SpeedDistance, payload, ec);
  }

  /**
   * @brief Buffered drive of motor 2 with signed speed and distance
   *        (command 42).
   * @param qpps Signed speed in quad pulses per second.
   * @param distance Distance in quad pulses (unsigned).
   * @param immediate If true, stop the currently-executing command, flush the
   *        buffer and run this command now; if false, queue it.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool buffered_drive_m2_speed_distance(int32_t qpps, uint32_t distance, bool immediate,
                                        std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_i32_be(payload, qpps);
    detail::append_u32_be(payload, distance);
    detail::append_u8(payload, immediate ? 1 : 0);
    return write_command(Command::BufferedM2SpeedDistance, payload, ec);
  }

  /**
   * @brief Buffered drive of both motors with signed speeds and distances
   *        (command 43).
   * @param qpps_m1 Signed speed for motor 1 in quad pulses per second.
   * @param distance_m1 Distance for motor 1 in quad pulses (unsigned).
   * @param qpps_m2 Signed speed for motor 2 in quad pulses per second.
   * @param distance_m2 Distance for motor 2 in quad pulses (unsigned).
   * @param immediate If true, stop the currently-executing command, flush the
   *        buffer and run this command now; if false, queue it.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool buffered_drive_speed_distance(int32_t qpps_m1, uint32_t distance_m1, int32_t qpps_m2,
                                     uint32_t distance_m2, bool immediate, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_i32_be(payload, qpps_m1);
    detail::append_u32_be(payload, distance_m1);
    detail::append_i32_be(payload, qpps_m2);
    detail::append_u32_be(payload, distance_m2);
    detail::append_u8(payload, immediate ? 1 : 0);
    return write_command(Command::BufferedM1M2SpeedDistance, payload, ec);
  }

  /**
   * @brief Buffered drive of motor 1 with acceleration, signed speed and
   *        distance (command 44).
   * @param accel Acceleration in qpps per second (unsigned).
   * @param qpps Signed speed in quad pulses per second.
   * @param distance Distance in quad pulses (unsigned).
   * @param immediate If true, stop the currently-executing command, flush the
   *        buffer and run this command now; if false, queue it.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool buffered_drive_m1_speed_accel_distance(uint32_t accel, int32_t qpps, uint32_t distance,
                                              bool immediate, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_u32_be(payload, accel);
    detail::append_i32_be(payload, qpps);
    detail::append_u32_be(payload, distance);
    detail::append_u8(payload, immediate ? 1 : 0);
    return write_command(Command::BufferedM1SpeedAccelDistance, payload, ec);
  }

  /**
   * @brief Buffered drive of motor 2 with acceleration, signed speed and
   *        distance (command 45).
   * @param accel Acceleration in qpps per second (unsigned).
   * @param qpps Signed speed in quad pulses per second.
   * @param distance Distance in quad pulses (unsigned).
   * @param immediate If true, stop the currently-executing command, flush the
   *        buffer and run this command now; if false, queue it.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool buffered_drive_m2_speed_accel_distance(uint32_t accel, int32_t qpps, uint32_t distance,
                                              bool immediate, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_u32_be(payload, accel);
    detail::append_i32_be(payload, qpps);
    detail::append_u32_be(payload, distance);
    detail::append_u8(payload, immediate ? 1 : 0);
    return write_command(Command::BufferedM2SpeedAccelDistance, payload, ec);
  }

  /**
   * @brief Buffered drive of both motors with a shared acceleration, signed
   *        speeds and distances (command 46).
   * @param accel Acceleration in qpps per second (unsigned, applies to both).
   * @param qpps_m1 Signed speed for motor 1 in quad pulses per second.
   * @param distance_m1 Distance for motor 1 in quad pulses (unsigned).
   * @param qpps_m2 Signed speed for motor 2 in quad pulses per second.
   * @param distance_m2 Distance for motor 2 in quad pulses (unsigned).
   * @param immediate If true, stop the currently-executing command, flush the
   *        buffer and run this command now; if false, queue it.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool buffered_drive_speed_accel_distance(uint32_t accel, int32_t qpps_m1, uint32_t distance_m1,
                                           int32_t qpps_m2, uint32_t distance_m2, bool immediate,
                                           std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_u32_be(payload, accel);
    detail::append_i32_be(payload, qpps_m1);
    detail::append_u32_be(payload, distance_m1);
    detail::append_i32_be(payload, qpps_m2);
    detail::append_u32_be(payload, distance_m2);
    detail::append_u8(payload, immediate ? 1 : 0);
    return write_command(Command::BufferedM1M2SpeedAccelDistance, payload, ec);
  }

  /**
   * @brief Read how many buffered commands are waiting per motor (command 47).
   * @param buffer_m1 Motor 1 buffer state: 0x80 = buffer empty / last command
   *        finished, 0 = last command is executing, 1-0x3F = commands waiting.
   * @param buffer_m2 Motor 2 buffer state (same encoding).
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_buffer_lengths(uint8_t &buffer_m1, uint8_t &buffer_m2, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    uint8_t data[2] = {};
    if (!read_command(Command::ReadBufferLengths, data, ec))
      return false;
    buffer_m1 = data[0];
    buffer_m2 = data[1];
    return true;
  }

  // ------------------------------ encoders ---------------------------------

  /**
   * @brief Read the motor 1 encoder count / position (command 16).
   * @param count Encoder count (quadrature: full 32-bit range; absolute:
   *        0-4095).
   * @param status Status bits: bit0 = underflow occurred (cleared on read),
   *        bit1 = direction (0 forward, 1 backward), bit2 = overflow occurred
   *        (cleared on read).
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_encoder_m1(uint32_t &count, uint8_t &status, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    return read_encoder(Command::ReadEncoderM1, count, status, ec);
  }

  /**
   * @brief Read the motor 2 encoder count / position (command 17).
   * @param count Encoder count (quadrature: full 32-bit range; absolute:
   *        0-4095).
   * @param status Status bits: bit0 = underflow occurred (cleared on read),
   *        bit1 = direction (0 forward, 1 backward), bit2 = overflow occurred
   *        (cleared on read).
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_encoder_m2(uint32_t &count, uint8_t &status, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    return read_encoder(Command::ReadEncoderM2, count, status, ec);
  }

  /**
   * @brief Read both encoder counters in one transaction (command 78).
   * @param count_m1 Motor 1 encoder count.
   * @param count_m2 Motor 2 encoder count.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_encoders(uint32_t &count_m1, uint32_t &count_m2, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    uint8_t data[8] = {};
    if (!read_command(Command::ReadEncoderCounters, data, ec))
      return false;
    count_m1 = detail::read_u32_be(data, 0);
    count_m2 = detail::read_u32_be(data, 4);
    return true;
  }

  /**
   * @brief Reset both quadrature encoder counters to zero (command 20).
   * @param ec Set on failure.
   * @return True on success.
   */
  bool reset_encoders(std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    return write_command(Command::ResetEncoders, {}, ec);
  }

  /**
   * @brief Read the motor 1 encoder speed in pulses per second (command 18).
   * @param qpps Speed in pulses per second (as reported by the controller).
   * @param direction 0 = forward, 1 = backward.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_encoder_speed_m1(int32_t &qpps, uint8_t &direction, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    return read_speed(Command::ReadEncoderSpeedM1, qpps, direction, ec);
  }

  /**
   * @brief Read the motor 2 encoder speed in pulses per second (command 19).
   * @param qpps Speed in pulses per second (as reported by the controller).
   * @param direction 0 = forward, 1 = backward.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_encoder_speed_m2(int32_t &qpps, uint8_t &direction, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    return read_speed(Command::ReadEncoderSpeedM2, qpps, direction, ec);
  }

  /**
   * @brief Read both instantaneous speeds (counts per second over the last
   *        1/300th of a second) in one transaction (command 79).
   * @param qpps_m1 Motor 1 instantaneous speed.
   * @param qpps_m2 Motor 2 instantaneous speed.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_ispeeds(int32_t &qpps_m1, int32_t &qpps_m2, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    uint8_t data[8] = {};
    if (!read_command(Command::ReadISpeedCounters, data, ec))
      return false;
    qpps_m1 = detail::read_i32_be(data, 0);
    qpps_m2 = detail::read_i32_be(data, 4);
    return true;
  }

  /**
   * @brief Read the encoder modes / pin assignments for both motors
   *        (command 91).
   * @param mode_m1 Motor 1 encoder mode.
   * @param mode_m2 Motor 2 encoder mode.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_encoder_modes(uint8_t &mode_m1, uint8_t &mode_m2, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    uint8_t data[2] = {};
    if (!read_command(Command::ReadEncoderModes, data, ec))
      return false;
    mode_m1 = data[0];
    mode_m2 = data[1];
    return true;
  }

  // ----------------------------- velocity PID ------------------------------

  /**
   * @brief Set the motor 1 velocity PID constants and QPPS (command 28).
   *
   * Gains are converted to the controller's 16.16 fixed-point representation
   * (value * 65536); the controller defaults correspond to P=1.0, I=0.5,
   * D=0.25, QPPS=44000.
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   * @param qpps Encoder speed (quad pulses per second) at 100% motor power.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool set_velocity_pid_m1(float p, float i, float d, uint32_t qpps, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    return set_velocity_pid(Command::SetVelocityPidM1, p, i, d, qpps, ec);
  }

  /**
   * @brief Set the motor 2 velocity PID constants and QPPS (command 29).
   *        See set_velocity_pid_m1() for the fixed-point conversion.
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   * @param qpps Encoder speed (quad pulses per second) at 100% motor power.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool set_velocity_pid_m2(float p, float i, float d, uint32_t qpps, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    return set_velocity_pid(Command::SetVelocityPidM2, p, i, d, qpps, ec);
  }

  /**
   * @brief Read the motor 1 velocity PID constants and QPPS (command 55).
   *        Fixed-point values are converted back to floats (divide by 65536).
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   * @param qpps Encoder speed (quad pulses per second) at 100% motor power.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_velocity_pid_m1(float &p, float &i, float &d, uint32_t &qpps, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    return read_velocity_pid(Command::ReadVelocityPidM1, p, i, d, qpps, ec);
  }

  /**
   * @brief Read the motor 2 velocity PID constants and QPPS (command 56).
   *        Fixed-point values are converted back to floats (divide by 65536).
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   * @param qpps Encoder speed (quad pulses per second) at 100% motor power.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_velocity_pid_m2(float &p, float &i, float &d, uint32_t &qpps, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    return read_velocity_pid(Command::ReadVelocityPidM2, p, i, d, qpps, ec);
  }

  // -------------------------------- telemetry ------------------------------

  /**
   * @brief Read the firmware version string (command 21).
   *
   * The controller returns up to 48 bytes terminated by a line feed and a NUL
   * character (e.g. "MCP266 2x60A v1.0.0"); the returned string has the
   * terminators stripped.
   * @param version The firmware / product version string.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_firmware_version(std::string &version, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    if (!transport_ok(ec))
      return false;
    const auto request = detail::build_read_request(
        config_.address, static_cast<uint8_t>(Command::ReadFirmwareVersion));
    if (!config_.write(request)) {
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    // variable-length reply: string bytes ... LF, NUL, then CRC16 (2 bytes)
    static constexpr size_t max_version_len = 48;
    std::vector<uint8_t> reply;
    reply.reserve(max_version_len + 2);
    bool terminated = false;
    while (reply.size() < max_version_len) {
      uint8_t b;
      if (!read_exact({&b, 1}, ec))
        return false;
      reply.push_back(b);
      if (reply.size() >= 2 && reply[reply.size() - 2] == '\n' && reply[reply.size() - 1] == '\0') {
        terminated = true;
        break;
      }
    }
    if (!terminated) {
      ec = std::make_error_code(std::errc::protocol_error);
      return false;
    }
    uint8_t crc[2] = {};
    if (!read_exact(crc, ec))
      return false;
    reply.insert(reply.end(), std::begin(crc), std::end(crc));
    if (!detail::validate_reply(config_.address, static_cast<uint8_t>(Command::ReadFirmwareVersion),
                                reply)) {
      logger_.error("read_firmware_version: bad reply CRC");
      ec = std::make_error_code(std::errc::protocol_error);
      return false;
    }
    // strip the LF + NUL terminators and the CRC
    version.assign(reply.begin(), reply.end() - 4);
    ec.clear();
    return true;
  }

  /**
   * @brief Read the main battery (B+/B-) voltage (command 24).
   * @param volts Voltage in volts (the controller reports tenths of a volt).
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_main_battery_voltage(float &volts, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    return read_tenths(Command::ReadMainBatteryVoltage, volts, ec);
  }

  /**
   * @brief Read the logic battery (LB+/LB-) voltage (command 25).
   * @param volts Voltage in volts (the controller reports tenths of a volt).
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_logic_battery_voltage(float &volts, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    return read_tenths(Command::ReadLogicBatteryVoltage, volts, ec);
  }

  /**
   * @brief Read the motor currents (command 49).
   * @param amps_m1 Motor 1 current in amps (the controller reports 10 mA
   *        units, i.e. value / 100).
   * @param amps_m2 Motor 2 current in amps.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_currents(float &amps_m1, float &amps_m2, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    uint8_t data[4] = {};
    if (!read_command(Command::ReadMotorCurrents, data, ec))
      return false;
    amps_m1 = static_cast<float>(detail::read_i16_be(data, 0)) / 100.0f;
    amps_m2 = static_cast<float>(detail::read_i16_be(data, 2)) / 100.0f;
    return true;
  }

  /**
   * @brief Read the motor PWM output values (command 48).
   * @param percent_m1 Motor 1 duty cycle in percent (-100 to +100; the
   *        controller reports +/-32767, i.e. value / 327.67).
   * @param percent_m2 Motor 2 duty cycle in percent.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_motor_pwms(float &percent_m1, float &percent_m2, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    uint8_t data[4] = {};
    if (!read_command(Command::ReadMotorPWMs, data, ec))
      return false;
    percent_m1 = static_cast<float>(detail::read_i16_be(data, 0)) / 327.67f;
    percent_m2 = static_cast<float>(detail::read_i16_be(data, 2)) / 327.67f;
    return true;
  }

  /**
   * @brief Read the board temperature (command 82).
   * @param degrees Temperature in degrees (the controller reports tenths of a
   *        degree).
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_temperature(float &degrees, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    return read_tenths(Command::ReadTemperature, degrees, ec);
  }

  /**
   * @brief Read the second board temperature (command 83, only on supported
   *        units).
   * @param degrees Temperature in degrees (the controller reports tenths of a
   *        degree).
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_temperature2(float &degrees, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    return read_tenths(Command::ReadTemperature2, degrees, ec);
  }

  /**
   * @brief Read the unit status bit mask (command 90). See Basicmicro::Status
   *        for the bit definitions.
   * @param status The status bit mask (0 = normal).
   * @param ec Set on failure.
   * @note The manual's status table defines 16-bit masks (up to 0x2000) but
   *       does not explicitly state the field width; this reads 2 bytes.
   * @return True on success.
   */
  bool read_status(uint16_t &status, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    uint8_t data[2] = {};
    if (!read_command(Command::ReadStatus, data, ec))
      return false;
    status = detail::read_u16_be(data, 0);
    return true;
  }

  // ------------------------------ management -------------------------------

  /**
   * @brief Write all settings to non-volatile memory (command 94) so they are
   *        reloaded on power-up.
   * @note Per the manual this request is sent WITHOUT a CRC ([Address, 94])
   *       but is still acknowledged with 0xFF.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool write_settings_to_eeprom(std::error_code &ec) {
    if (!transport_ok(ec))
      return false;
    std::scoped_lock lk(mutex_);
    const auto request = detail::build_read_request(
        config_.address, static_cast<uint8_t>(Command::WriteSettingsToEeprom));
    if (!config_.write(request)) {
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    return read_ack(ec);
  }

  /**
   * @brief Reset an E-Stop condition (command 200). Does nothing unless the
   *        E-Stop reset has been unlocked (manual command 201).
   * @param ec Set on failure.
   * @return True on success.
   */
  bool e_stop_reset(std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    return write_command(Command::EStopReset, {}, ec);
  }

protected:
  /// Validate that both transport functions were configured. Calling an empty
  /// std::function throws std::bad_function_call, which would violate this
  /// component's no-exceptions contract -- so every transaction entry point
  /// checks here first and fails with invalid_argument instead.
  bool transport_ok(std::error_code &ec) {
    if (!config_.write || !config_.read) {
      logger_.error("Config::write and Config::read must both be set");
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    return true;
  }

  /// Read exactly buf.size() bytes, looping on the user read function until
  /// the configured timeout elapses. Sets ec and returns false on timeout.
  bool read_exact(std::span<uint8_t> buf, std::error_code &ec) {
    const auto deadline = std::chrono::steady_clock::now() + config_.timeout;
    size_t got = 0;
    while (got < buf.size()) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline)
        break;
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
      const size_t n =
          config_.read(buf.subspan(got), std::max(remaining, std::chrono::milliseconds(1)));
      if (n == 0)
        break; // the read function timed out
      got += n;
    }
    if (got < buf.size()) {
      logger_.debug("timed out reading reply ({}/{} bytes)", got, buf.size());
      ec = std::make_error_code(std::errc::timed_out);
      return false;
    }
    ec.clear();
    return true;
  }

  /// Read and check the single 0xFF ACK byte of a write command.
  bool read_ack(std::error_code &ec) {
    uint8_t ack = 0;
    if (!read_exact({&ack, 1}, ec))
      return false;
    if (ack != detail::kBasicmicroAck) {
      logger_.error("bad ACK byte: 0x{:02X}", ack);
      ec = std::make_error_code(std::errc::protocol_error);
      return false;
    }
    ec.clear();
    return true;
  }

  /// Run one write-command transaction: [addr, cmd, payload, CRC] -> 0xFF.
  /// The caller must hold mutex_.
  bool write_command(Command cmd, std::span<const uint8_t> payload, std::error_code &ec) {
    if (!transport_ok(ec))
      return false;
    const auto packet =
        detail::build_write_packet(config_.address, static_cast<uint8_t>(cmd), payload);
    logger_.debug("write command {} ({} byte packet)", static_cast<int>(cmd), packet.size());
    if (!config_.write(packet)) {
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    return read_ack(ec);
  }

  /// Run one read-command transaction: [addr, cmd] -> data + CRC. The reply
  /// data (excluding CRC) is written into @p data, whose size determines how
  /// many data bytes are expected. The caller must hold mutex_.
  bool read_command(Command cmd, std::span<uint8_t> data, std::error_code &ec) {
    if (!transport_ok(ec))
      return false;
    const auto request = detail::build_read_request(config_.address, static_cast<uint8_t>(cmd));
    if (!config_.write(request)) {
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    // reply = data bytes + 2 CRC bytes
    std::vector<uint8_t> reply(data.size() + 2);
    if (!read_exact(reply, ec))
      return false;
    if (!detail::validate_reply(config_.address, static_cast<uint8_t>(cmd), reply)) {
      logger_.error("read command {}: bad reply CRC", static_cast<int>(cmd));
      ec = std::make_error_code(std::errc::protocol_error);
      return false;
    }
    std::copy(reply.begin(), reply.end() - 2, data.begin());
    ec.clear();
    return true;
  }

  /// Shared implementation for commands 16/17 (count + status byte).
  bool read_encoder(Command cmd, uint32_t &count, uint8_t &status, std::error_code &ec) {
    uint8_t data[5] = {};
    if (!read_command(cmd, data, ec))
      return false;
    count = detail::read_u32_be(data, 0);
    status = data[4];
    return true;
  }

  /// Shared implementation for commands 18/19/30/31 (speed + direction byte).
  bool read_speed(Command cmd, int32_t &qpps, uint8_t &direction, std::error_code &ec) {
    uint8_t data[5] = {};
    if (!read_command(cmd, data, ec))
      return false;
    qpps = detail::read_i32_be(data, 0);
    direction = data[4];
    return true;
  }

  /// Shared implementation for the 2-byte tenths-of-a-unit reads (24/25/82/83).
  bool read_tenths(Command cmd, float &value, std::error_code &ec) {
    uint8_t data[2] = {};
    if (!read_command(cmd, data, ec))
      return false;
    value = static_cast<float>(detail::read_u16_be(data, 0)) / 10.0f;
    return true;
  }

  /// Shared implementation for commands 28/29. Wire order is D, P, I, QPPS.
  bool set_velocity_pid(Command cmd, float p, float i, float d, uint32_t qpps,
                        std::error_code &ec) {
    std::vector<uint8_t> payload;
    detail::append_u32_be(payload, static_cast<uint32_t>(d * detail::kBasicmicroPidScale));
    detail::append_u32_be(payload, static_cast<uint32_t>(p * detail::kBasicmicroPidScale));
    detail::append_u32_be(payload, static_cast<uint32_t>(i * detail::kBasicmicroPidScale));
    detail::append_u32_be(payload, qpps);
    return write_command(cmd, payload, ec);
  }

  /// Shared implementation for commands 55/56. Wire order is P, I, D, QPPS.
  bool read_velocity_pid(Command cmd, float &p, float &i, float &d, uint32_t &qpps,
                         std::error_code &ec) {
    uint8_t data[16] = {};
    if (!read_command(cmd, data, ec))
      return false;
    p = static_cast<float>(detail::read_u32_be(data, 0)) / detail::kBasicmicroPidScale;
    i = static_cast<float>(detail::read_u32_be(data, 4)) / detail::kBasicmicroPidScale;
    d = static_cast<float>(detail::read_u32_be(data, 8)) / detail::kBasicmicroPidScale;
    qpps = detail::read_u32_be(data, 12);
    return true;
  }

  Config config_;

  /// Serializes complete transactions (request write + ACK/reply read) so
  /// concurrent callers cannot interleave packets on the shared serial line.
  std::mutex mutex_;
};

} // namespace espp
