#pragma once

#include <chrono>
#include <functional>
#include <string>
#include <system_error>
#include <thread>

#include "base_component.hpp"
#include "canopen_client.hpp"

namespace espp {

/// \brief CiA 402 (DS402) drive-profile helper, layered on a CanopenClient.
/// \details Wraps the standard CiA 402 objects (controlword 0x6040, statusword
///          0x6041, modes of operation 0x6060/0x6061, target/actual velocity &
///          position, profile velocity/acceleration/deceleration) and the
///          power-drive-system state machine: decode the statusword into a
///          State, walk the enable sequence (Shutdown -> Switch On -> Enable
///          Operation) with statusword polling and timeout, quick-stop, and
///          fault reset (rising edge of controlword bit 7). Supports the
///          profile velocity (pv), profile position (pp), and homing (hm)
///          modes, including the new-set-point handshake (controlword bits
///          4/5/6) for profile position moves.
///
///          All communication goes through the referenced CanopenClient's SDO
///          channel, so the same threading rules apply (calls block, and
///          frames must be delivered to the client from another task).
///
/// \section ds402_ex0 DS402 Drive Example
/// \snippet canopen_example.cpp canopen example
class Ds402Drive : public BaseComponent {
public:
  using State = detail::ds402::State;                 ///< CiA 402 drive state.
  using OperatingMode = detail::ds402::OperatingMode; ///< CiA 402 mode of operation.

  /// \brief Human-readable name for a CiA 402 drive state (e.g. for logging).
  static const char *to_string(State state) { return detail::ds402::state_to_string(state); }

  /// \brief Decode a raw CiA 402 statusword (object 0x6041) into a drive state.
  /// \details Useful for decoding a statusword you already have (e.g. from a
  ///          cached read or a TPDO) without another SDO round-trip.
  static State state_from_statusword(uint16_t statusword) {
    return detail::ds402::decode_state(statusword);
  }

  /// \brief Configuration for the Ds402Drive.
  struct Config {
    std::chrono::milliseconds state_timeout{
        1000}; ///< Timeout for each state transition / mode change to take effect.
    std::chrono::milliseconds poll_period{10}; ///< Statusword polling period.
    uint16_t object_offset{0}; ///< Added to device-profile object indices (0x6000-0x6FFF) to
                               ///< select an axis on a multi-axis drive: 0 for the first/only
                               ///< axis, 0x800 for a second axis (e.g. Basicmicro MCP266 M2,
                               ///< whose objects mirror M1 at +0x800). Communication and
                               ///< identity objects (< 0x6000) are shared by the device and
                               ///< never offset.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /// \brief Create a DS402 drive helper.
  /// \param client The CANopen client for the drive's node. Must outlive this object.
  /// \param config The configuration.
  explicit Ds402Drive(CanopenClient &client, const Config &config)
      : BaseComponent("Ds402Drive", config.log_level)
      , client_(client)
      , state_timeout_(config.state_timeout)
      , poll_period_(config.poll_period)
      , object_offset_(config.object_offset) {
    // Validate the axis offset once, up front, rather than silently falling
    // back to the un-offset (axis 1) index later: an offset large enough to
    // push a device-profile object past the 16-bit index space is a
    // misconfiguration, so reject it loudly and disable offsetting.
    if (object_offset_ > detail::ds402::MAX_AXIS_OBJECT_OFFSET) {
      logger_.error("object_offset 0x{:04X} exceeds the maximum 0x{:04X} (it would push a "
                    "device-profile index past 0x{:04X}); ignoring it and using axis 1",
                    object_offset_, detail::ds402::MAX_AXIS_OBJECT_OFFSET,
                    detail::ds402::OBJ_INDEX_MAX);
      object_offset_ = 0;
    }
  }

  /// \brief Create a DS402 drive helper with the default configuration.
  /// \param client The CANopen client for the drive's node. Must outlive this object.
  explicit Ds402Drive(CanopenClient &client)
      : Ds402Drive(client, Config{}) {}

  /// @name Standard object accessors
  /// @{

  /// \brief Read the device type (object 0x1000). \param ec Set on failure. \return The value.
  uint32_t get_device_type(std::error_code &ec) {
    return client_.read_u32(detail::ds402::OBJ_DEVICE_TYPE, 0, ec);
  }
  /// \brief Read the error register (object 0x1001). \param ec Set on failure. \return The value.
  uint8_t get_error_register(std::error_code &ec) {
    return client_.read_u8(detail::ds402::OBJ_ERROR_REGISTER, 0, ec);
  }
  /// \brief Read the manufacturer device name (object 0x1008, segmented upload).
  /// \param ec Set on failure. \return The device name string.
  std::string get_device_name(std::error_code &ec) {
    return client_.read_string(detail::ds402::OBJ_DEVICE_NAME, 0, ec);
  }
  /// \brief Read the identity vendor id (object 0x1018:1). \param ec Set on failure.
  /// \return The value.
  uint32_t get_vendor_id(std::error_code &ec) {
    return client_.read_u32(detail::ds402::OBJ_IDENTITY, 1, ec);
  }
  /// \brief Read the identity product code (object 0x1018:2). \param ec Set on failure.
  /// \return The value.
  uint32_t get_product_code(std::error_code &ec) {
    return client_.read_u32(detail::ds402::OBJ_IDENTITY, 2, ec);
  }
  /// \brief Read the identity revision number (object 0x1018:3). \param ec Set on failure.
  /// \return The value.
  uint32_t get_revision_number(std::error_code &ec) {
    return client_.read_u32(detail::ds402::OBJ_IDENTITY, 3, ec);
  }
  /// \brief Read the identity serial number (object 0x1018:4). \param ec Set on failure.
  /// \return The value.
  uint32_t get_serial_number(std::error_code &ec) {
    return client_.read_u32(detail::ds402::OBJ_IDENTITY, 4, ec);
  }

  /// \brief Write the controlword (object 0x6040). \param controlword Value to write.
  /// \param ec Set on failure. \return True on success.
  bool set_controlword(uint16_t controlword, std::error_code &ec) {
    logger_.debug("controlword <- 0x{:04X}", controlword);
    return client_.write_u16(axis_object(detail::ds402::OBJ_CONTROLWORD), 0, controlword, ec);
  }
  /// \brief Read the statusword (object 0x6041). \param ec Set on failure. \return The value.
  uint16_t get_statusword(std::error_code &ec) {
    return client_.read_u16(axis_object(detail::ds402::OBJ_STATUSWORD), 0, ec);
  }
  /// \brief Read the velocity actual value (object 0x606C). \param ec Set on failure.
  /// \return The value.
  int32_t get_velocity_actual(std::error_code &ec) {
    return client_.read_i32(axis_object(detail::ds402::OBJ_VELOCITY_ACTUAL), 0, ec);
  }
  /// \brief Read the position actual value (object 0x6064). \param ec Set on failure.
  /// \return The value.
  int32_t get_position_actual(std::error_code &ec) {
    return client_.read_i32(axis_object(detail::ds402::OBJ_POSITION_ACTUAL), 0, ec);
  }
  /// \brief Write the profile velocity (object 0x6081). \param velocity Value to write.
  /// \param ec Set on failure. \return True on success.
  bool set_profile_velocity(uint32_t velocity, std::error_code &ec) {
    return client_.write_u32(axis_object(detail::ds402::OBJ_PROFILE_VELOCITY), 0, velocity, ec);
  }
  /// \brief Write the profile acceleration (object 0x6083). \param acceleration Value to write.
  /// \param ec Set on failure. \return True on success.
  bool set_profile_acceleration(uint32_t acceleration, std::error_code &ec) {
    return client_.write_u32(axis_object(detail::ds402::OBJ_PROFILE_ACCELERATION), 0, acceleration,
                             ec);
  }
  /// \brief Write the profile deceleration (object 0x6084). \param deceleration Value to write.
  /// \param ec Set on failure. \return True on success.
  bool set_profile_deceleration(uint32_t deceleration, std::error_code &ec) {
    return client_.write_u32(axis_object(detail::ds402::OBJ_PROFILE_DECELERATION), 0, deceleration,
                             ec);
  }

  /// @}

  /// @name State machine
  /// @{

  /// \brief Read the statusword and decode the CiA 402 drive state.
  /// \param ec Set on failure (returns State::Unknown).
  /// \return The decoded state.
  State get_state(std::error_code &ec) {
    const auto statusword = get_statusword(ec);
    if (ec) {
      return State::Unknown;
    }
    return detail::ds402::decode_state(statusword);
  }

  /// \brief Walk the drive to Operation Enabled.
  /// \details Issues Shutdown (0x0006) -> Switch On (0x0007) -> Enable
  ///          Operation (0x000F), polling the statusword after each command
  ///          until the corresponding state is reached or the configured
  ///          state_timeout expires. If the drive is in Fault, call
  ///          fault_reset() first.
  /// \param ec Set on communication failure or transition timeout.
  /// \return True once the drive reports Operation Enabled.
  bool enable_operation(std::error_code &ec) {
    auto state = get_state(ec);
    if (ec) {
      return false;
    }
    logger_.info("enable_operation: starting from state '{}'",
                 detail::ds402::state_to_string(state));
    if (state == State::Fault || state == State::FaultReactionActive) {
      logger_.error("enable_operation: drive is in fault; call fault_reset() first");
      ec = std::make_error_code(std::errc::operation_not_permitted);
      return false;
    }
    if (state != State::ReadyToSwitchOn && state != State::SwitchedOn &&
        state != State::OperationEnabled) {
      if (!command_and_wait(detail::ds402::CW_SHUTDOWN, State::ReadyToSwitchOn, ec)) {
        return false;
      }
      state = State::ReadyToSwitchOn;
    }
    if (state == State::ReadyToSwitchOn) {
      if (!command_and_wait(detail::ds402::CW_SWITCH_ON, State::SwitchedOn, ec)) {
        return false;
      }
      state = State::SwitchedOn;
    }
    if (state == State::SwitchedOn) {
      if (!command_and_wait(detail::ds402::CW_ENABLE_OPERATION, State::OperationEnabled, ec)) {
        return false;
      }
    }
    logger_.info("enable_operation: drive is in Operation Enabled");
    return true;
  }

  /// \brief Disable the drive (Shutdown command -> Ready to switch on, power stage off).
  /// \param ec Set on communication failure or transition timeout.
  /// \return True once the drive reports Ready to switch on.
  bool disable(std::error_code &ec) {
    return command_and_wait(detail::ds402::CW_SHUTDOWN, State::ReadyToSwitchOn, ec);
  }

  /// \brief Issue a quick stop (controlword 0x0002).
  /// \details Depending on the drive's quick-stop option code it transitions to
  ///          Quick Stop Active or directly to Switch On Disabled, so this does
  ///          not poll for a specific target state.
  /// \param ec Set on communication failure.
  /// \return True on success.
  bool quick_stop(std::error_code &ec) { return set_controlword(detail::ds402::CW_QUICK_STOP, ec); }

  /// \brief Reset a drive fault (rising edge on controlword bit 7).
  /// \details Writes controlword 0x0000 then 0x0080, then polls until the
  ///          drive leaves the Fault state.
  /// \param ec Set on communication failure or if the fault persists.
  /// \return True once the drive is no longer in Fault.
  bool fault_reset(std::error_code &ec) {
    if (!set_controlword(0x0000, ec)) {
      return false;
    }
    if (!set_controlword(detail::ds402::CW_FAULT_RESET, ec)) {
      return false;
    }
    return wait_for_state(
        [](State s) { return s != State::Fault && s != State::FaultReactionActive; },
        "fault cleared", ec);
  }

  /// \brief Set the mode of operation (object 0x6060) and verify via 0x6061.
  /// \param mode The mode to select (e.g. OperatingMode::ProfileVelocity).
  /// \param ec Set on communication failure or if the drive does not report the
  ///        mode within the state timeout.
  /// \return True once modes-of-operation-display matches.
  bool set_mode(OperatingMode mode, std::error_code &ec) {
    if (!client_.write_i8(axis_object(detail::ds402::OBJ_MODES_OF_OPERATION), 0,
                          static_cast<int8_t>(mode), ec)) {
      return false;
    }
    const auto deadline = std::chrono::steady_clock::now() + state_timeout_;
    do {
      const auto display =
          client_.read_i8(axis_object(detail::ds402::OBJ_MODES_OF_OPERATION_DISPLAY), 0, ec);
      if (ec) {
        return false;
      }
      if (display == static_cast<int8_t>(mode)) {
        return true;
      }
      std::this_thread::sleep_for(poll_period_);
    } while (std::chrono::steady_clock::now() < deadline);
    logger_.error("set_mode: drive did not report mode {} within {} ms", static_cast<int>(mode),
                  state_timeout_.count());
    ec = std::make_error_code(std::errc::timed_out);
    return false;
  }

  /// \brief Read the mode of operation display (object 0x6061).
  /// \param ec Set on failure. \return The reported mode.
  int8_t get_mode_display(std::error_code &ec) {
    return client_.read_i8(axis_object(detail::ds402::OBJ_MODES_OF_OPERATION_DISPLAY), 0, ec);
  }

  /// @}

  /// @name Motion
  /// @{

  /// \brief Write the target velocity (object 0x60FF; profile velocity mode).
  /// \param velocity Target velocity in device units.
  /// \param ec Set on failure. \return True on success.
  bool set_target_velocity(int32_t velocity, std::error_code &ec) {
    return client_.write_i32(axis_object(detail::ds402::OBJ_TARGET_VELOCITY), 0, velocity, ec);
  }

  /// \brief Command a profile-position move (object 0x607A + new-set-point handshake).
  /// \details Writes the target position, then raises controlword bit 4 (new
  ///          set-point) with bit 5 (change set immediately) and bit 6
  ///          (relative) as requested, waits for the drive to acknowledge via
  ///          statusword bit 12, and clears bit 4 again. The drive must already
  ///          be in Operation Enabled in profile position mode.
  /// \param position Target position in device units.
  /// \param ec Set on communication failure or acknowledge timeout.
  /// \param immediate If true, the drive starts the new move immediately (bit 5).
  /// \param relative If true, the target is relative to the current position (bit 6).
  /// \return True once the set-point was acknowledged and bit 4 released.
  bool set_target_position(int32_t position, std::error_code &ec, bool immediate = true,
                           bool relative = false) {
    if (!client_.write_i32(axis_object(detail::ds402::OBJ_TARGET_POSITION), 0, position, ec)) {
      return false;
    }
    uint16_t controlword = detail::ds402::CW_ENABLE_OPERATION;
    if (immediate) {
      controlword |= detail::ds402::CW_BIT_CHANGE_SET_IMMEDIATELY;
    }
    if (relative) {
      controlword |= detail::ds402::CW_BIT_RELATIVE;
    }
    // raise the new-set-point bit (4)...
    if (!set_controlword(controlword | detail::ds402::CW_BIT_NEW_SETPOINT, ec)) {
      return false;
    }
    // ...wait for set-point-acknowledge (statusword bit 12)...
    const auto deadline = std::chrono::steady_clock::now() + state_timeout_;
    while (true) {
      const auto statusword = get_statusword(ec);
      if (ec) {
        return false;
      }
      if ((statusword & detail::ds402::SW_BIT_SETPOINT_ACKNOWLEDGE) != 0) {
        break;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        logger_.error("set_target_position: no set-point acknowledge within {} ms",
                      state_timeout_.count());
        ec = std::make_error_code(std::errc::timed_out);
        return false;
      }
      std::this_thread::sleep_for(poll_period_);
    }
    // ...release the new-set-point bit again...
    if (!set_controlword(controlword, ec)) {
      return false;
    }
    // ...and confirm the drive clears set-point-acknowledge before returning,
    // so the next setpoint's rising edge is unambiguous (some drives hold the
    // acknowledge bit until bit 4 is released; starting a new handshake with
    // it still high can be silently ignored).
    const auto ack_deadline = std::chrono::steady_clock::now() + state_timeout_;
    while (true) {
      const auto statusword = get_statusword(ec);
      if (ec) {
        return false;
      }
      if ((statusword & detail::ds402::SW_BIT_SETPOINT_ACKNOWLEDGE) == 0) {
        return true;
      }
      if (std::chrono::steady_clock::now() >= ack_deadline) {
        logger_.error("set_target_position: set-point acknowledge did not clear within {} ms",
                      state_timeout_.count());
        ec = std::make_error_code(std::errc::timed_out);
        return false;
      }
      std::this_thread::sleep_for(poll_period_);
    }
  }

  /// \brief Check whether the drive reports target reached (statusword bit 10).
  /// \param ec Set on failure. \return True if the target is reached.
  bool is_target_reached(std::error_code &ec) {
    return (get_statusword(ec) & detail::ds402::SW_BIT_TARGET_REACHED) != 0;
  }

  /// @}

protected:
  /// Write \p controlword, then poll until the drive reports \p target_state.
  bool command_and_wait(uint16_t controlword, State target_state, std::error_code &ec) {
    if (!set_controlword(controlword, ec)) {
      return false;
    }
    return wait_for_state([target_state](State s) { return s == target_state; },
                          detail::ds402::state_to_string(target_state), ec);
  }

  /// Poll get_state() until \p predicate is satisfied or state_timeout_ expires.
  bool wait_for_state(const std::function<bool(State)> &predicate, const char *description,
                      std::error_code &ec) {
    const auto deadline = std::chrono::steady_clock::now() + state_timeout_;
    while (true) {
      const auto state = get_state(ec);
      if (ec) {
        return false;
      }
      if (predicate(state)) {
        return true;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        logger_.error("timed out after {} ms waiting for '{}' (state is '{}')",
                      state_timeout_.count(), description, detail::ds402::state_to_string(state));
        ec = std::make_error_code(std::errc::timed_out);
        return false;
      }
      std::this_thread::sleep_for(poll_period_);
    }
  }

  /// Apply the configured axis offset to a CiA 402 device-profile object index.
  /// Only objects the helper knows to be axis-relative -- the standard
  /// device-profile range (OBJ_DEVICE_PROFILE_MIN..OBJ_DEVICE_PROFILE_MAX) --
  /// are offset; every other index (communication / identity objects below the
  /// range, and anything above it) is not offset and is returned unchanged.
  /// object_offset_ is validated in the constructor, so the addition never
  /// overflows the 16-bit index space here.
  uint16_t axis_object(uint16_t index) const {
    return detail::ds402::apply_axis_offset(index, object_offset_);
  }

  CanopenClient &client_;
  std::chrono::milliseconds state_timeout_;
  std::chrono::milliseconds poll_period_;
  uint16_t object_offset_;
};

} // namespace espp
