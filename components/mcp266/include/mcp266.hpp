#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <system_error>
#include <thread>

#include "base_component.hpp"
#include "canopen_client.hpp"
#include "detail/mcp266_core.hpp"
#include "ds402.hpp"
#include "motor_controller.hpp"

namespace espp {

/// \brief Dual-channel controller for a Basicmicro MCP266 (RoboClaw family)
///        brushed-DC motor driver over CANopen.
/// \details Layered on a CanopenClient (like Ds402Drive), so it is
///          transport-agnostic: the application owns the CAN transport, feeds
///          received frames to the client's process_frame(), and the client's
///          node id selects the MCP266. Both motor channels (M1, M2) are
///          driven symmetrically; M2's CiA 402 objects mirror M1's at +0x800,
///          handled through Ds402Drive's object offset.
///
///          Position control uses the standard CiA 402 profile position mode
///          and is the supported, validated capability. The MCP266's
///          control-loop parameters are NOT standard CiA 402 objects: the MCP
///          mirrors its packet-serial command set into the manufacturer region
///          at object index 0x2000 + command number, which this class uses to
///          configure and read back the position PID (commands 61-64), set or
///          zero the encoder counts (22/23, 20), issue the manufacturer
///          speed/duty commands (32/33, 35/36), and read telemetry (24, 82).
///          See detail/mcp266_core.hpp.
///
///          \b Important: two device quirks must be handled, both done by
///          configure_position_loop():
///           - The position PID's MinPos/MaxPos clamp defaults to [0, 0],
///             which forces every position target to zero.
///           - The setter (commands 61/62) uses field order D, P, I while the
///             readback (63/64) uses P, I, D, so a naive read-modify-write of
///             the record moves P into the D slot and zeros P.
///          The MCP reverts to its EEPROM configuration on power-up, so call
///          configure_position_loop() once per boot before commanding moves.
///
///          \b Note: the manufacturer speed/duty command mirror
///          (drive_speed / drive_duty) is implemented but does NOT produce
///          motion on the MCP266 firmware tested (the command is accepted but
///          the velocity generator stays idle). Use position mode for motion.
///
/// \section mcp266_ex0 MCP266 Example
/// \snippet mcp266_example.cpp mcp266 example
class Mcp266 : public BaseComponent {
public:
  /// \brief Motor channel selector (shared across the espp motor drivers).
  using Axis = MotorAxis;

  /// \brief Configuration for the Mcp266 controller.
  struct Config {
    std::chrono::milliseconds state_timeout{1000};        ///< Per-axis CiA 402 transition timeout.
    std::chrono::milliseconds poll_period{25};            ///< Statusword polling period.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /// \brief Create an MCP266 controller.
  /// \param client The CANopen client for the MCP266's node. Must outlive this
  ///        object, and its process_frame() must be driven from another task
  ///        (e.g. the transport's receive task) per CanopenClient's contract.
  /// \param config The configuration.
  explicit Mcp266(CanopenClient &client, const Config &config)
      : BaseComponent("Mcp266", config.log_level)
      , client_(client)
      , m1_(client, detail::mcp266::axis_m1(), "M1", config)
      , m2_(client, detail::mcp266::axis_m2(), "M2", config) {}

  /// \brief Create an MCP266 controller with the default configuration.
  explicit Mcp266(CanopenClient &client)
      : Mcp266(client, Config{}) {}

  /// \brief NMT-start the node and clear any latched CiA 402 faults on both
  ///        axes. Call once after the transport and client are up.
  /// \param ec Set on failure.
  /// \return True on success.
  bool start(std::error_code &ec) {
    ec.clear();
    if (!client_.nmt_start(ec)) {
      // Fail fast: reset_faults() below clears ec, so a swallowed NMT failure
      // would otherwise be reported as overall success.
      logger_.error("NMT start failed: {}", ec.message());
      return false;
    }
    return reset_faults(ec);
  }

  /// \brief Clear any latched CiA 402 fault on both axes. \param ec Set on
  /// failure. \return True on success.
  bool reset_faults(std::error_code &ec) {
    ec.clear();
    for (AxisState *a : {&m1_, &m2_}) {
      const auto state = a->drive.get_state(ec);
      if (ec) {
        return false;
      }
      if (state == Ds402Drive::State::Fault || state == Ds402Drive::State::FaultReactionActive) {
        logger_.warn("{}: clearing fault", a->name);
        if (!a->drive.fault_reset(ec)) {
          return false;
        }
      }
    }
    return true;
  }

  /// \brief Attempt an E-stop reset (mirrored packet-serial command 200 at
  ///        0x20C8). Harmless when nothing is latched. \param ec Set on
  ///        failure. \return True if accepted.
  bool reset_estop(std::error_code &ec) {
    ec.clear();
    // Command 200 has no payload, so the SDO scalar width is unknown; try the
    // common ones.
    bool ok = client_.write_u8(detail::mcp266::kEStopResetObject, 0, 1, ec);
    if (!ok) {
      ec.clear(); // don't let the first attempt's error leak into the retry
      ok = client_.write_u32(detail::mcp266::kEStopResetObject, 0, 1, ec);
    }
    logger_.info("E-stop reset {}", ok ? "accepted" : "rejected");
    return ok; // on success ec is clear; on failure ec holds the last error
  }

  /// @name Position control (CiA 402 profile position mode)
  /// @{

  /// \brief Configure an axis's position loop for use: widen the MinPos/MaxPos
  ///        clamp (factory [0, 0] forces every target to zero) and, only if the
  ///        drive's stored position P gain reads back as zero, seed a non-zero
  ///        P so the loop produces output. The record is written through the
  ///        setter's D, P, I field order and the clamp verified via the
  ///        readback. The MCP reverts to EEPROM on power-up, so call once per
  ///        boot.
  /// \note \p fallback_p is a coarse starting value used ONLY when the drive
  ///       has no stored P gain; it is not motor-tuned. For good motion, tune
  ///       the position PID in Basicmicro Motion Studio (or pass a value
  ///       appropriate for your motor / encoder) -- an unsuitable P gain can
  ///       leave the axis sluggish or make it oscillate. A drive whose P gain
  ///       is already non-zero keeps its stored gains untouched.
  /// \param axis The motor channel.
  /// \param min_pos Minimum commandable position.
  /// \param max_pos Maximum commandable position.
  /// \param fallback_p Position P gain to seed when the stored gain is zero.
  /// \param ec Set on failure.
  /// \return True on success.
  bool configure_position_loop(Axis axis, int32_t min_pos, int32_t max_pos, int32_t fallback_p,
                               std::error_code &ec) {
    ec.clear();
    if (!check_axis(axis, ec))
      return false;
    AxisState &a = axis_state(axis);
    std::array<int32_t, 7> readback{};
    if (!read_position_pid_raw(a, readback, ec)) {
      return false;
    }
    // readback order is [P, I, D, MaxI, Deadzone, MinPos, MaxPos]
    if (readback[0] == 0) {
      readback[0] = fallback_p; // seed P only if the record has none
      logger_.warn("{}: stored position P gain was 0; seeding coarse fallback {} (tune for your "
                   "motor)",
                   a.name, fallback_p);
    }
    readback[5] = min_pos;
    readback[6] = max_pos;
    if (!write_position_pid_raw(a, detail::mcp266::position_pid_readback_to_setter(readback), ec)) {
      return false;
    }
    // verify via the readback's field order (min/max are subs 6/7 there too)
    int32_t got_min = 0;
    int32_t got_max = 0;
    if (!read_position_limits(axis, got_min, got_max, ec) || got_min != min_pos ||
        got_max != max_pos) {
      logger_.error("{}: position clamp did not take (read [{}, {}], wanted [{}, {}])", a.name,
                    got_min, got_max, min_pos, max_pos);
      ec = std::make_error_code(std::errc::protocol_error);
      return false;
    }
    logger_.info("{}: position loop configured (P={}, clamp=[{}, {}])", a.name, readback[0],
                 min_pos, max_pos);
    return true;
  }

  /// \brief Configure an axis's position loop using the default coarse fallback
  ///        P gain (see the fallback_p overload). Convenience for the common case.
  bool configure_position_loop(Axis axis, int32_t min_pos, int32_t max_pos, std::error_code &ec) {
    return configure_position_loop(axis, min_pos, max_pos, kDefaultPositionP, ec);
  }

  /// \brief Read an axis's position PID record (mirrored command 63/64). Gains
  ///        are converted to floats (the record stores them x1024), matching
  ///        espp::Basicmicro::read_position_pid().
  /// \param axis The motor channel.
  /// \param p Out: proportional gain.
  /// \param i Out: integral gain.
  /// \param d Out: derivative gain.
  /// \param max_i Out: maximum integral windup.
  /// \param deadzone Out: deadzone in encoder counts.
  /// \param min_pos Out: minimum commandable position (the clamp).
  /// \param max_pos Out: maximum commandable position (the clamp).
  /// \param ec Set on failure.
  /// \return True on success.
  bool read_position_pid(Axis axis, float &p, float &i, float &d, uint32_t &max_i,
                         uint32_t &deadzone, int32_t &min_pos, int32_t &max_pos,
                         std::error_code &ec) {
    ec.clear();
    if (!check_axis(axis, ec))
      return false;
    std::array<int32_t, 7> readback{};
    if (!read_position_pid_raw(axis_state(axis), readback, ec)) {
      return false;
    }
    // gain / MaxI / Deadzone fields are unsigned on the record; the i32 read
    // only carried their bit pattern
    p = detail::mcp266::position_gain_from_bits(readback[0]);
    i = detail::mcp266::position_gain_from_bits(readback[1]);
    d = detail::mcp266::position_gain_from_bits(readback[2]);
    max_i = static_cast<uint32_t>(readback[3]);
    deadzone = static_cast<uint32_t>(readback[4]);
    min_pos = readback[5];
    max_pos = readback[6];
    return true;
  }

  /// \brief Write an axis's whole position PID record (mirrored command
  ///        61/62), taking care of the setter's D, P, I field order. Gains are
  ///        floats, stored x1024 (rounded to nearest and clamped non-negative,
  ///        as espp::Basicmicro::set_position_pid() does). Prefer
  ///        configure_position_loop() when only the clamp needs setting; this
  ///        is for installing tuned gains.
  /// \param axis The motor channel.
  /// \param p Proportional gain.
  /// \param i Integral gain.
  /// \param d Derivative gain.
  /// \param max_i Maximum integral windup.
  /// \param deadzone Deadzone in encoder counts.
  /// \param min_pos Minimum commandable position (the clamp).
  /// \param max_pos Maximum commandable position (the clamp).
  /// \param ec Set on failure.
  /// \return True on success.
  bool set_position_pid(Axis axis, float p, float i, float d, uint32_t max_i, uint32_t deadzone,
                        int32_t min_pos, int32_t max_pos, std::error_code &ec) {
    ec.clear();
    if (!check_axis(axis, ec))
      return false;
    if (min_pos > max_pos) {
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    const AxisState &a = axis_state(axis);
    using detail::mcp266::position_gain_bits;
    using detail::mcp266::scale_position_gain;
    // the unsigned fields travel as 4-byte bit patterns in the i32 slots
    const std::array<int32_t, 7> readback_order{position_gain_bits(scale_position_gain(p)),
                                                position_gain_bits(scale_position_gain(i)),
                                                position_gain_bits(scale_position_gain(d)),
                                                position_gain_bits(max_i),
                                                position_gain_bits(deadzone),
                                                min_pos,
                                                max_pos};
    if (!write_position_pid_raw(a, detail::mcp266::position_pid_readback_to_setter(readback_order),
                                ec)) {
      return false;
    }
    logger_.info("{}: position PID written (P={}, I={}, D={}, clamp=[{}, {}])", a.name,
                 static_cast<uint32_t>(readback_order[0]), static_cast<uint32_t>(readback_order[1]),
                 static_cast<uint32_t>(readback_order[2]), min_pos, max_pos);
    return true;
  }

  /// \brief Read just the MinPos/MaxPos clamp of an axis's position PID record
  ///        (two SDOs instead of the seven of read_position_pid()). Useful as a
  ///        cheap check that the controller still holds the configuration it
  ///        was given: it reverts to its EEPROM (factory clamp [0, 0]) on a
  ///        power cycle.
  /// \param axis The motor channel.
  /// \param min_pos Out: minimum commandable position.
  /// \param max_pos Out: maximum commandable position.
  /// \param ec Set on failure.
  /// \return True on success.
  bool read_position_limits(Axis axis, int32_t &min_pos, int32_t &max_pos, std::error_code &ec) {
    ec.clear();
    if (!check_axis(axis, ec))
      return false;
    const AxisState &a = axis_state(axis);
    min_pos = client_.read_i32(a.objects.position_pid_get, 6, ec);
    if (!ec) {
      max_pos = client_.read_i32(a.objects.position_pid_get, 7, ec);
    }
    if (ec) {
      logger_.error("{}: position limits read 0x{:04X} failed: {}", a.name,
                    a.objects.position_pid_get, ec.message());
      return false;
    }
    return true;
  }

  /// \brief Set the CiA 402 software position limits (object 0x607D:1/:2) for an
  ///        axis — a per-move envelope enforced by the drive's trajectory
  ///        generator. Distinct from configure_position_loop()'s min/max, which
  ///        writes the manufacturer position-PID MinPos/MaxPos clamp.
  /// \param axis The motor channel.
  /// \param min_pos Lower limit.
  /// \param max_pos Upper limit.
  /// \param ec Set on failure.
  /// \return True on success.
  bool set_software_position_limits(Axis axis, int32_t min_pos, int32_t max_pos,
                                    std::error_code &ec) {
    ec.clear();
    if (!check_axis(axis, ec))
      return false;
    if (min_pos > max_pos) {
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    const uint16_t obj = detail::ds402::apply_axis_offset(
        detail::ds402::OBJ_SOFTWARE_POSITION_LIMIT, axis_state(axis).objects.object_offset);
    return client_.write_i32(obj, 1, min_pos, ec) && client_.write_i32(obj, 2, max_pos, ec);
  }

  /// \brief Command a profile-position move: enable the axis, set the motion
  ///        profile, and issue the target with the new-set-point handshake.
  /// \param axis The motor channel.
  /// \param target_position Absolute target position (encoder counts).
  /// \param profile_velocity Cruise velocity.
  /// \param profile_acceleration Acceleration.
  /// \param profile_deceleration Deceleration.
  /// \param ec Set on failure.
  /// \return True once the set-point is accepted.
  bool move_to_position(Axis axis, int32_t target_position, uint32_t profile_velocity,
                        uint32_t profile_acceleration, uint32_t profile_deceleration,
                        std::error_code &ec) {
    ec.clear();
    if (!check_axis(axis, ec))
      return false;
    AxisState &a = axis_state(axis);
    if (!enable(a, Ds402Drive::OperatingMode::ProfilePosition, ec)) {
      return false;
    }
    if (!(a.drive.set_profile_acceleration(profile_acceleration, ec) &&
          a.drive.set_profile_deceleration(profile_deceleration, ec) &&
          a.drive.set_profile_velocity(profile_velocity, ec))) {
      return false;
    }
    return a.drive.set_target_position(target_position, ec);
  }

  /// @}

  /// @name Manufacturer speed / duty command mirror
  /// @note Accepted by the drive but inert on the MCP266 firmware tested; kept
  ///       for completeness and in case a firmware update activates them.
  /// @{

  /// \brief Closed-loop speed via the mirrored packet-serial command (35/36).
  bool drive_speed(Axis axis, int32_t qpps, std::error_code &ec) {
    ec.clear();
    if (!check_axis(axis, ec))
      return false;
    AxisState &a = axis_state(axis);
    if (qpps != 0 && !enable(a, Ds402Drive::OperatingMode::ProfileVelocity, ec)) {
      return false;
    }
    return client_.write_i32(a.objects.drive_speed, 0, qpps, ec);
  }

  /// \brief Open-loop duty via the mirrored packet-serial command (32/33).
  bool drive_duty(Axis axis, int16_t duty, std::error_code &ec) {
    ec.clear();
    if (!check_axis(axis, ec))
      return false;
    AxisState &a = axis_state(axis);
    if (duty != 0 && !enable(a, Ds402Drive::OperatingMode::ProfileVelocity, ec)) {
      return false;
    }
    return client_.write_i16(a.objects.drive_duty, 0, duty, ec);
  }

  /// @}

  /// @name Feedback
  /// @{

  /// \brief Read the actual position (0x6064 / 0x6864).
  /// \param axis Channel.
  /// \param count Out: encoder counts.
  /// \param ec Set on failure.
  /// \return True on success.
  bool read_encoder(Axis axis, int32_t &count, std::error_code &ec) {
    ec.clear();
    if (!check_axis(axis, ec))
      return false;
    count = axis_state(axis).drive.get_position_actual(ec);
    return !ec;
  }
  /// \brief Set an axis's encoder count (mirrored packet-serial command 22/23).
  ///        For a quadrature encoder this defines the count at the current
  ///        position, e.g. to home against a limit switch or to restore a
  ///        remembered position after power-up. Takes effect at once; the
  ///        position loop then sees the new count.
  /// \param axis Channel.
  /// \param count The count to install.
  /// \param ec Set on failure.
  /// \return True on success.
  bool set_encoder(Axis axis, int32_t count, std::error_code &ec) {
    ec.clear();
    if (!check_axis(axis, ec))
      return false;
    const AxisState &a = axis_state(axis);
    if (!client_.write_i32(a.objects.encoder_set, 0, count, ec)) {
      logger_.error("{}: set encoder 0x{:04X} rejected: {}", a.name, a.objects.encoder_set,
                    ec.message());
      return false;
    }
    logger_.info("{}: encoder count set to {}", a.name, count);
    return true;
  }
  /// \brief Zero both encoder counters (mirrored packet-serial command 20),
  ///        matching espp::Basicmicro::reset_encoders().
  /// \param ec Set on failure.
  /// \return True on success.
  bool reset_encoders(std::error_code &ec) {
    ec.clear();
    // Command 20 has no payload, so the SDO scalar width is unknown; try the
    // common ones (as reset_estop() does).
    bool ok = client_.write_u8(detail::mcp266::kResetEncodersObject, 0, 1, ec);
    if (!ok) {
      ec.clear();
      ok = client_.write_u32(detail::mcp266::kResetEncodersObject, 0, 1, ec);
    }
    logger_.info("encoder reset {}", ok ? "accepted" : "rejected");
    return ok;
  }
  /// \brief Read the actual velocity (0x606C / 0x686C).
  /// \param axis Channel.
  /// \param qpps Out: counts/s.
  /// \param ec Set on failure.
  /// \return True on success.
  bool read_speed(Axis axis, int32_t &qpps, std::error_code &ec) {
    ec.clear();
    if (!check_axis(axis, ec))
      return false;
    qpps = axis_state(axis).drive.get_velocity_actual(ec);
    return !ec;
  }
  /// \brief Read the CiA 402 statusword (0x6041 / 0x6841).
  /// \param axis Channel.
  /// \param statusword Out.
  /// \param ec Set on failure.
  /// \return True on success.
  bool read_statusword(Axis axis, uint16_t &statusword, std::error_code &ec) {
    ec.clear();
    if (!check_axis(axis, ec))
      return false;
    statusword = axis_state(axis).drive.get_statusword(ec);
    return !ec;
  }
  /// \brief Read the decoded CiA 402 drive state of an axis (from its statusword).
  /// \param axis Channel.
  /// \param state Out: the power-drive-system state.
  /// \param ec Set on failure.
  /// \return True on success.
  bool get_state(Axis axis, Ds402Drive::State &state, std::error_code &ec) {
    ec.clear();
    if (!check_axis(axis, ec))
      return false;
    state = axis_state(axis).drive.get_state(ec);
    return !ec;
  }
  /// \brief Whether an axis reports "target reached" (statusword bit 10) — the
  ///        authoritative arrival signal for profile moves.
  /// \param axis Channel.
  /// \param reached Out.
  /// \param ec Set on failure.
  /// \return True on success.
  bool is_target_reached(Axis axis, bool &reached, std::error_code &ec) {
    ec.clear();
    if (!check_axis(axis, ec))
      return false;
    reached = axis_state(axis).drive.is_target_reached(ec);
    return !ec;
  }

  /// @}

  /// @name Device telemetry
  /// @{

  /// \brief Read the main battery voltage (mirrored command 24).
  /// \param volts Out: volts.
  /// \param ec Set on failure.
  /// \return True on success.
  bool read_main_battery_voltage(float &volts, std::error_code &ec) {
    ec.clear();
    volts = static_cast<float>(client_.read_u16(detail::mcp266::kMainBatteryObject, 0, ec)) / 10.0f;
    return !ec;
  }
  /// \brief Read the board temperature (mirrored command 82).
  /// \param temp_c Out: degrees C.
  /// \param ec Set on failure.
  /// \return True on success.
  bool read_temperature(float &temp_c, std::error_code &ec) {
    ec.clear();
    temp_c =
        static_cast<float>(client_.read_u16(detail::mcp266::kTemperatureObject, 0, ec)) / 10.0f;
    return !ec;
  }
  /// \brief Read the standard device type (0x1000) and name (0x1008).
  bool read_device_info(std::string &device_name, uint32_t &device_type, std::error_code &ec) {
    ec.clear();
    device_type = client_.read_u32(0x1000, 0, ec);
    if (ec) {
      return false;
    }
    device_name = client_.read_string(0x1008, 0, ec);
    return !ec;
  }

  /// @}

  /// \brief Access an axis's underlying Ds402Drive for advanced CiA 402 use.
  /// \param axis The motor channel.
  /// \return Pointer to the axis drive helper, or nullptr if \p axis is not a
  ///         valid channel (M1 / M2) -- so an axis decoded from an untrusted
  ///         byte cannot silently return the M2 drive.
  Ds402Drive *drive(Axis axis) {
    if (axis != Axis::M1 && axis != Axis::M2)
      return nullptr;
    return &axis_state(axis).drive;
  }

private:
  /// Coarse fallback position P gain, used only when the drive's stored gain
  /// reads back as zero (see configure_position_loop()). It is a non-tuned
  /// starting point that produces motion out of the box, not a good gain for
  /// any particular motor; callers should tune and pass their own. 15491 is
  /// ~15.1 in the MCP's position-PID fixed-point representation (x1024).
  static constexpr int32_t kDefaultPositionP = 15491; // = 0x3C83

  /// The MCP266 does not echo the requested mode in 0x6061, so after writing the
  /// mode of operation enable() waits this fixed settle time before reading the
  /// state rather than polling the (unchanging) mode display.
  static constexpr auto kModeSettle = std::chrono::milliseconds(25);

  /// Per-axis state: the manufacturer object addresses and a Ds402Drive whose
  /// object offset selects M1 (0) or M2 (0x800).
  struct AxisState {
    detail::mcp266::AxisObjects objects;
    Ds402Drive drive;
    const char *name;

    AxisState(CanopenClient &client, const detail::mcp266::AxisObjects &objs, const char *n,
              const Config &cfg)
        : objects(objs)
        , drive(client, Ds402Drive::Config{.state_timeout = cfg.state_timeout,
                                           .poll_period = cfg.poll_period,
                                           .object_offset = objs.object_offset,
                                           .log_level = cfg.log_level})
        , name(n) {}
  };

  /// Reject an out-of-range axis selector. MotorAxis is a two-value enum, but a
  /// value decoded from an untrusted byte could be neither M1 nor M2, and
  /// axis_state()'s dispatch would otherwise silently select M2. Public axis
  /// methods call this before any SDO I/O. Sets ec = invalid_argument on failure.
  static bool check_axis(Axis axis, std::error_code &ec) {
    if (axis == Axis::M1 || axis == Axis::M2)
      return true;
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }

  AxisState &axis_state(Axis axis) { return axis == Axis::M1 ? m1_ : m2_; }

  /// Read the seven-field position PID record in the readback's order
  /// [P, I, D, MaxI, Deadzone, MinPos, MaxPos] (subindices 1..7 of command 63/64).
  /// The first five fields are unsigned on the device; the i32 read carries
  /// their 4-byte bit pattern, which the callers reinterpret.
  bool read_position_pid_raw(const AxisState &a, std::array<int32_t, 7> &readback,
                             std::error_code &ec) {
    for (uint8_t sub = 1; sub <= 7; ++sub) {
      readback[sub - 1] = client_.read_i32(a.objects.position_pid_get, sub, ec);
      if (ec) {
        logger_.error("{}: position PID read 0x{:04X}:{} failed: {}", a.name,
                      a.objects.position_pid_get, sub, ec.message());
        return false;
      }
    }
    return true;
  }

  /// Write the seven-field record already in the setter's order
  /// [D, P, I, MaxI, Deadzone, MinPos, MaxPos] (subindices 1..7 of command 61/62).
  bool write_position_pid_raw(const AxisState &a, const std::array<int32_t, 7> &setter,
                              std::error_code &ec) {
    for (uint8_t sub = 1; sub <= 7; ++sub) {
      if (!client_.write_i32(a.objects.position_pid_set, sub, setter[sub - 1], ec)) {
        logger_.error("{}: position PID write 0x{:04X}:{} rejected: {}", a.name,
                      a.objects.position_pid_set, sub, ec.message());
        return false;
      }
    }
    return true;
  }

  /// Write the axis mode of operation directly (the MCP does not echo the
  /// requested mode in 0x6061, so Ds402Drive::set_mode() -- which verifies the
  /// display -- would time out), clear any fault, and walk to Operation
  /// Enabled.
  bool enable(AxisState &a, Ds402Drive::OperatingMode mode, std::error_code &ec) {
    const uint16_t mode_obj = detail::ds402::apply_axis_offset(
        detail::ds402::OBJ_MODES_OF_OPERATION, a.objects.object_offset);
    if (!client_.write_i8(mode_obj, 0, static_cast<int8_t>(mode), ec)) {
      logger_.error("{}: failed to set mode: {}", a.name, ec.message());
      return false;
    }
    std::this_thread::sleep_for(kModeSettle);
    const auto state = a.drive.get_state(ec);
    if (ec) {
      return false;
    }
    if (state == Ds402Drive::State::Fault || state == Ds402Drive::State::FaultReactionActive) {
      if (!a.drive.fault_reset(ec)) {
        return false;
      }
    }
    return a.drive.enable_operation(ec);
  }

  CanopenClient &client_;
  AxisState m1_;
  AxisState m2_;
};

static_assert(
    MotorController<Mcp266>,
    "Mcp266 must satisfy the espp::MotorController concept (kept in sync with Basicmicro)");

} // namespace espp
