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
#include "motor_controller.hpp"

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
  /// Motor channel selector, shared with the other espp motor drivers (e.g.
  /// espp::Mcp266) so generic code can command either transport by axis.
  /// \see espp::MotorController
  using Axis = MotorAxis;

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
   * @brief Drive one motor with a signed duty cycle (commands 32 / 33).
   * @param axis Motor channel to drive.
   * @param duty Signed duty, -32767 to +32767 (= -100% to +100%).
   * @param ec Set on failure.
   * @return True on success.
   */
  bool drive_duty(Axis axis, int16_t duty, std::error_code &ec) {
    if (!check_axis(axis, ec))
      return false;
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_i16_be(payload, duty);
    return write_command(axis == Axis::M1 ? Command::DriveM1SignedDuty : Command::DriveM2SignedDuty,
                         payload, ec);
  }

  /**
   * @brief Drive both motors with signed duty cycles in one packet (command 34).
   * @param duty_m1 Signed duty for motor 1, -32767 to +32767.
   * @param duty_m2 Signed duty for motor 2, -32767 to +32767.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool drive_both_duty(int16_t duty_m1, int16_t duty_m2, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_i16_be(payload, duty_m1);
    detail::append_i16_be(payload, duty_m2);
    return write_command(Command::DriveM1M2SignedDuty, payload, ec);
  }

  // ------------------------ closed-loop speed drive ------------------------

  /**
   * @brief Drive one motor at a signed speed in quadrature pulses per second
   *        (commands 35 / 36). Requires an encoder and tuned velocity PID.
   * @param axis Motor channel to drive.
   * @param qpps Signed speed in quad pulses per second.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool drive_speed(Axis axis, int32_t qpps, std::error_code &ec) {
    if (!check_axis(axis, ec))
      return false;
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_i32_be(payload, qpps);
    return write_command(
        axis == Axis::M1 ? Command::DriveM1SignedSpeed : Command::DriveM2SignedSpeed, payload, ec);
  }

  /**
   * @brief Drive both motors at signed speeds in one packet (command 37).
   * @param qpps_m1 Signed speed for motor 1 in quad pulses per second.
   * @param qpps_m2 Signed speed for motor 2 in quad pulses per second.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool drive_both_speed(int32_t qpps_m1, int32_t qpps_m2, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_i32_be(payload, qpps_m1);
    detail::append_i32_be(payload, qpps_m2);
    return write_command(Command::DriveM1M2SignedSpeed, payload, ec);
  }

  /**
   * @brief Drive one motor at a signed speed with an acceleration ramp
   *        (commands 38 / 39).
   * @param axis Motor channel to drive.
   * @param accel Acceleration in qpps per second (unsigned).
   * @param qpps Signed target speed in quad pulses per second.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool drive_speed_accel(Axis axis, uint32_t accel, int32_t qpps, std::error_code &ec) {
    if (!check_axis(axis, ec))
      return false;
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_u32_be(payload, accel);
    detail::append_i32_be(payload, qpps);
    return write_command(axis == Axis::M1 ? Command::DriveM1SignedSpeedAccel
                                          : Command::DriveM2SignedSpeedAccel,
                         payload, ec);
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
  bool drive_both_speed_accel(uint32_t accel, int32_t qpps_m1, int32_t qpps_m2,
                              std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_u32_be(payload, accel);
    detail::append_i32_be(payload, qpps_m1);
    detail::append_i32_be(payload, qpps_m2);
    return write_command(Command::DriveM1M2SignedSpeedAccel, payload, ec);
  }

  // -------------------------- buffered motion ------------------------------

  /**
   * @brief Buffered drive of one motor with signed speed and distance
   *        (commands 41 / 42).
   * @param axis Motor channel to drive.
   * @param qpps Signed speed in quad pulses per second.
   * @param distance Distance in quad pulses (unsigned).
   * @param immediate If true, stop the currently-executing command, flush the
   *        buffer and run this command now; if false, queue it (up to 64
   *        commands per motor buffer).
   * @param ec Set on failure.
   * @return True on success.
   */
  bool buffered_drive_speed_distance(Axis axis, int32_t qpps, uint32_t distance, bool immediate,
                                     std::error_code &ec) {
    if (!check_axis(axis, ec))
      return false;
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_i32_be(payload, qpps);
    detail::append_u32_be(payload, distance);
    detail::append_u8(payload, immediate ? 1 : 0);
    return write_command(axis == Axis::M1 ? Command::BufferedM1SpeedDistance
                                          : Command::BufferedM2SpeedDistance,
                         payload, ec);
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
  bool buffered_drive_both_speed_distance(int32_t qpps_m1, uint32_t distance_m1, int32_t qpps_m2,
                                          uint32_t distance_m2, bool immediate,
                                          std::error_code &ec) {
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
   * @brief Buffered drive of one motor with acceleration, signed speed and
   *        distance (commands 44 / 45).
   * @param axis Motor channel to drive.
   * @param accel Acceleration in qpps per second (unsigned).
   * @param qpps Signed speed in quad pulses per second.
   * @param distance Distance in quad pulses (unsigned).
   * @param immediate If true, stop the currently-executing command, flush the
   *        buffer and run this command now; if false, queue it.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool buffered_drive_speed_accel_distance(Axis axis, uint32_t accel, int32_t qpps,
                                           uint32_t distance, bool immediate, std::error_code &ec) {
    if (!check_axis(axis, ec))
      return false;
    std::scoped_lock lk(mutex_);
    std::vector<uint8_t> payload;
    detail::append_u32_be(payload, accel);
    detail::append_i32_be(payload, qpps);
    detail::append_u32_be(payload, distance);
    detail::append_u8(payload, immediate ? 1 : 0);
    return write_command(axis == Axis::M1 ? Command::BufferedM1SpeedAccelDistance
                                          : Command::BufferedM2SpeedAccelDistance,
                         payload, ec);
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
  bool buffered_drive_both_speed_accel_distance(uint32_t accel, int32_t qpps_m1,
                                                uint32_t distance_m1, int32_t qpps_m2,
                                                uint32_t distance_m2, bool immediate,
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
   * @brief Read one motor's encoder count / position (commands 16 / 17).
   * @param axis Motor channel to read.
   * @param count Signed encoder count (quadrature counters wrap through the full
   *        32-bit range; an absolute encoder reports 0-4095).
   * @param ec Set on failure.
   * @return True on success.
   * @note The controller reports the counter as raw 32 bits; it is returned here
   *       as a signed int32 so a quadrature encoder run in reverse reads as a
   *       negative count. Use the overload taking a @c status out-parameter for
   *       the underflow / direction / overflow flags.
   */
  bool read_encoder(Axis axis, int32_t &count, std::error_code &ec) {
    uint8_t status = 0;
    return read_encoder(axis, count, status, ec);
  }

  /**
   * @brief Read one motor's encoder count / position and status byte
   *        (commands 16 / 17).
   * @param axis Motor channel to read.
   * @param count Signed encoder count (see the two-argument overload).
   * @param status Status bits: bit0 = underflow occurred (cleared on read),
   *        bit1 = direction (0 forward, 1 backward), bit2 = overflow occurred
   *        (cleared on read).
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_encoder(Axis axis, int32_t &count, uint8_t &status, std::error_code &ec) {
    if (!check_axis(axis, ec))
      return false;
    std::scoped_lock lk(mutex_);
    uint32_t raw = 0;
    if (!read_count_raw(axis == Axis::M1 ? Command::ReadEncoderM1 : Command::ReadEncoderM2, raw,
                        status, ec))
      return false;
    count = static_cast<int32_t>(raw);
    return true;
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
   * @brief Read one motor's encoder speed in pulses per second
   *        (commands 18 / 19).
   * @param axis Motor channel to read.
   * @param qpps Speed in pulses per second (as reported by the controller).
   * @param ec Set on failure.
   * @return True on success.
   * @note Commands 18/19 report an unsigned speed magnitude plus a separate
   *       direction byte; this overload folds that direction into the sign so
   *       reverse motion reads as a negative speed (the signed-speed contract of
   *       espp::MotorController). Use the overload taking a @c direction
   *       out-parameter to read the raw magnitude and the 0 = forward /
   *       1 = backward flag separately.
   */
  bool read_speed(Axis axis, int32_t &qpps, std::error_code &ec) {
    uint8_t direction = 0;
    if (!read_speed(axis, qpps, direction, ec))
      return false;
    if (direction != 0)
      qpps = -qpps;
    return true;
  }

  /**
   * @brief Read one motor's encoder speed and direction (commands 18 / 19).
   * @param axis Motor channel to read.
   * @param qpps Speed in pulses per second (as reported by the controller).
   * @param direction 0 = forward, 1 = backward.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_speed(Axis axis, int32_t &qpps, uint8_t &direction, std::error_code &ec) {
    if (!check_axis(axis, ec))
      return false;
    std::scoped_lock lk(mutex_);
    return read_speed_raw(axis == Axis::M1 ? Command::ReadEncoderSpeedM1
                                           : Command::ReadEncoderSpeedM2,
                          qpps, direction, ec);
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
   * @brief Set one motor's velocity PID constants and QPPS (commands 28 / 29).
   *
   * Gains are converted to the controller's 16.16 fixed-point representation
   * (value * 65536); the controller defaults correspond to P=1.0, I=0.5,
   * D=0.25, QPPS=44000.
   * @param axis Motor channel to configure.
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   * @param qpps Encoder speed (quad pulses per second) at 100% motor power.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool set_velocity_pid(Axis axis, float p, float i, float d, uint32_t qpps, std::error_code &ec) {
    if (!check_axis(axis, ec))
      return false;
    std::scoped_lock lk(mutex_);
    return set_velocity_pid_raw(axis == Axis::M1 ? Command::SetVelocityPidM1
                                                 : Command::SetVelocityPidM2,
                                p, i, d, qpps, ec);
  }

  /**
   * @brief Read one motor's velocity PID constants and QPPS (commands 55 / 56).
   *        Fixed-point values are converted back to floats (divide by 65536).
   * @param axis Motor channel to read.
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   * @param qpps Encoder speed (quad pulses per second) at 100% motor power.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_velocity_pid(Axis axis, float &p, float &i, float &d, uint32_t &qpps,
                         std::error_code &ec) {
    if (!check_axis(axis, ec))
      return false;
    std::scoped_lock lk(mutex_);
    return read_velocity_pid_raw(axis == Axis::M1 ? Command::ReadVelocityPidM1
                                                  : Command::ReadVelocityPidM2,
                                 p, i, d, qpps, ec);
  }

  /**
   * @brief Set one motor's position PID constants (commands 61 / 62).
   *
   * The position loop has seven constants: P, I, D gains (transferred scaled
   * by 1024), MaxI (integral windup limit), Deadzone (in encoder counts), and
   * MinPos / MaxPos (the position range the loop will command; a target
   * outside it is clamped). The factory default for every constant is zero,
   * so position commands (65-67) produce no motion until these are set. Note
   * the wire order for this command is D, P, I (unlike the P, I, D read order
   * of command 63).
   * @param axis Motor channel to configure.
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   * @param max_i Maximum integral windup.
   * @param deadzone Deadzone in encoder counts.
   * @param min_pos Minimum commandable position.
   * @param max_pos Maximum commandable position.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool set_position_pid(Axis axis, float p, float i, float d, uint32_t max_i, uint32_t deadzone,
                        int32_t min_pos, int32_t max_pos, std::error_code &ec) {
    if (!check_axis(axis, ec))
      return false;
    std::scoped_lock lk(mutex_);
    return set_position_pid_raw(axis == Axis::M1 ? Command::SetPositionPidM1
                                                 : Command::SetPositionPidM2,
                                p, i, d, max_i, deadzone, min_pos, max_pos, ec);
  }

  /**
   * @brief Read one motor's position PID constants (commands 63 / 64).
   *        Gains are converted back to floats (divide by 1024). The reply
   *        order is P, I, D (unlike the D, P, I write order of command 61).
   * @param axis Motor channel to read.
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   * @param max_i Maximum integral windup.
   * @param deadzone Deadzone in encoder counts.
   * @param min_pos Minimum commandable position.
   * @param max_pos Maximum commandable position.
   * @param ec Set on failure.
   * @return True on success.
   */
  bool read_position_pid(Axis axis, float &p, float &i, float &d, uint32_t &max_i,
                         uint32_t &deadzone, int32_t &min_pos, int32_t &max_pos,
                         std::error_code &ec) {
    if (!check_axis(axis, ec))
      return false;
    std::scoped_lock lk(mutex_);
    return read_position_pid_raw(axis == Axis::M1 ? Command::ReadPositionPidM1
                                                  : Command::ReadPositionPidM2,
                                 p, i, d, max_i, deadzone, min_pos, max_pos, ec);
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
   *        for the bit definitions (the manual documents the low 16 bits).
   * @param status The status bit mask (0 = normal).
   * @param ec Set on failure.
   * @note The manual leaves the field width unstated, but current MCP firmware
   *       returns a 32-bit status (Basicmicro's official Arduino library reads
   *       it with Read4). This reads 4 bytes first; if that transaction fails
   *       (older firmware replying 16-bit makes the reply end mid-read), the
   *       device's 10 ms packet-clear gap has already elapsed via the timeout,
   *       so a single 16-bit retry is performed for legacy-firmware units.
   * @return True on success.
   */
  bool read_status(uint32_t &status, std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    {
      uint8_t data[4] = {};
      std::error_code ec32;
      if (read_command(Command::ReadStatus, data, ec32)) {
        status = detail::read_u32_be(data, 0);
        // The 32-bit attempt used a local ec32; honor the "true => ec cleared"
        // contract so a caller reusing ec sees success.
        ec.clear();
        return true;
      }
    }
    // Legacy retry: the failed 32-bit attempt consumed the full receive
    // timeout (>= the 10 ms packet-clear window), so the controller's packet
    // buffer is clean and any short reply was fully drained from the host.
    logger_.warn("32-bit status read failed; retrying as 16-bit (legacy firmware)");
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
  bool reset_estop(std::error_code &ec) {
    std::scoped_lock lk(mutex_);
    return write_command(Command::EStopReset, {}, ec);
  }

protected:
  /// Reject an out-of-range axis selector before any I/O. MotorAxis is a
  /// two-value enum, but a value decoded from an untrusted byte could be neither
  /// M1 nor M2, and the `axis == Axis::M1 ? ... : ...` dispatch would otherwise
  /// silently target M2. Sets ec = invalid_argument and returns false.
  static bool check_axis(Axis axis, std::error_code &ec) {
    if (axis == Axis::M1 || axis == Axis::M2)
      return true;
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }

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
  bool read_count_raw(Command cmd, uint32_t &count, uint8_t &status, std::error_code &ec) {
    uint8_t data[5] = {};
    if (!read_command(cmd, data, ec))
      return false;
    count = detail::read_u32_be(data, 0);
    status = data[4];
    return true;
  }

  /// Shared implementation for commands 18/19/30/31 (speed + direction byte).
  bool read_speed_raw(Command cmd, int32_t &qpps, uint8_t &direction, std::error_code &ec) {
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
  bool set_velocity_pid_raw(Command cmd, float p, float i, float d, uint32_t qpps,
                            std::error_code &ec) {
    std::vector<uint8_t> payload;
    // Route through scale_pid_gain (rounds; guards negative -> uint32 wrap and
    // NaN/inf -> UB in std::llround) just like the position path — a raw
    // static_cast of the float product bypassed those guards.
    detail::append_u32_be(payload, detail::scale_pid_gain(d, detail::kBasicmicroPidScale));
    detail::append_u32_be(payload, detail::scale_pid_gain(p, detail::kBasicmicroPidScale));
    detail::append_u32_be(payload, detail::scale_pid_gain(i, detail::kBasicmicroPidScale));
    detail::append_u32_be(payload, qpps);
    return write_command(cmd, payload, ec);
  }

  /// Shared implementation for commands 55/56. Wire order is P, I, D, QPPS.
  bool read_velocity_pid_raw(Command cmd, float &p, float &i, float &d, uint32_t &qpps,
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

  /// Shared implementation for commands 61/62. Wire order is
  /// D, P, I, MaxI, Deadzone, MinPos, MaxPos (P/I/D scaled by 1024).
  bool set_position_pid_raw(Command cmd, float p, float i, float d, uint32_t max_i,
                            uint32_t deadzone, int32_t min_pos, int32_t max_pos,
                            std::error_code &ec) {
    std::vector<uint8_t> payload;
    payload.reserve(28); // fixed 7 x 4 bytes; avoid incremental reallocations
    detail::append_u32_be(payload, detail::scale_pid_gain(d, detail::kBasicmicroPositionPidScale));
    detail::append_u32_be(payload, detail::scale_pid_gain(p, detail::kBasicmicroPositionPidScale));
    detail::append_u32_be(payload, detail::scale_pid_gain(i, detail::kBasicmicroPositionPidScale));
    detail::append_u32_be(payload, max_i);
    detail::append_u32_be(payload, deadzone);
    detail::append_i32_be(payload, min_pos);
    detail::append_i32_be(payload, max_pos);
    return write_command(cmd, payload, ec);
  }

  /// Shared implementation for commands 63/64. Reply order is
  /// P, I, D, MaxI, Deadzone, MinPos, MaxPos (P/I/D scaled by 1024).
  bool read_position_pid_raw(Command cmd, float &p, float &i, float &d, uint32_t &max_i,
                             uint32_t &deadzone, int32_t &min_pos, int32_t &max_pos,
                             std::error_code &ec) {
    uint8_t data[28] = {};
    if (!read_command(cmd, data, ec))
      return false;
    p = static_cast<float>(detail::read_u32_be(data, 0)) / detail::kBasicmicroPositionPidScale;
    i = static_cast<float>(detail::read_u32_be(data, 4)) / detail::kBasicmicroPositionPidScale;
    d = static_cast<float>(detail::read_u32_be(data, 8)) / detail::kBasicmicroPositionPidScale;
    max_i = detail::read_u32_be(data, 12);
    deadzone = detail::read_u32_be(data, 16);
    min_pos = detail::read_i32_be(data, 20);
    max_pos = detail::read_i32_be(data, 24);
    return true;
  }

  Config config_;

  /// Serializes complete transactions (request write + ACK/reply read) so
  /// concurrent callers cannot interleave packets on the shared serial line.
  std::mutex mutex_;
};

static_assert(MotorController<Basicmicro>,
              "Basicmicro must satisfy the shared espp::MotorController interface");

} // namespace espp
