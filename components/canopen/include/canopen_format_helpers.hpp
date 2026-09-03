#pragma once

#include <string_view>

#include "format.hpp"

#include "detail/canopen_core.hpp"

// libfmt formatters for the CANopen / CiA 402 enums so application code can print
// them directly -- e.g. `logger.info("Drive state: {}", state)` -- instead of
// calling a to-string helper inline at every call site.

/// \brief fmt formatter for a CiA 402 drive state (espp::Ds402Drive::State).
template <> struct fmt::formatter<espp::detail::ds402::State> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(espp::detail::ds402::State state, FormatContext &ctx) const {
    return fmt::formatter<std::string_view>::format(espp::detail::ds402::state_to_string(state),
                                                    ctx);
  }
};

/// \brief fmt formatter for a CiA 402 mode of operation
/// (espp::Ds402Drive::OperatingMode).
template <>
struct fmt::formatter<espp::detail::ds402::OperatingMode> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(espp::detail::ds402::OperatingMode mode, FormatContext &ctx) const {
    using espp::detail::ds402::OperatingMode;
    std::string_view s = "Unknown";
    switch (mode) {
    case OperatingMode::ProfilePosition:
      s = "Profile position";
      break;
    case OperatingMode::ProfileVelocity:
      s = "Profile velocity";
      break;
    case OperatingMode::ProfileTorque:
      s = "Profile torque";
      break;
    case OperatingMode::Homing:
      s = "Homing";
      break;
    }
    return fmt::formatter<std::string_view>::format(s, ctx);
  }
};

/// \brief fmt formatter for an NMT node state (espp::CanopenClient::NmtState).
template <>
struct fmt::formatter<espp::detail::canopen::NmtState> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(espp::detail::canopen::NmtState state, FormatContext &ctx) const {
    using espp::detail::canopen::NmtState;
    std::string_view s = "Unknown";
    switch (state) {
    case NmtState::BootUp:
      s = "Boot-up";
      break;
    case NmtState::Stopped:
      s = "Stopped";
      break;
    case NmtState::Operational:
      s = "Operational";
      break;
    case NmtState::PreOperational:
      s = "Pre-operational";
      break;
    case NmtState::Unknown:
      break;
    }
    return fmt::formatter<std::string_view>::format(s, ctx);
  }
};

/// \brief fmt formatter for an NMT master command (espp::CanopenClient::NmtCommand).
template <>
struct fmt::formatter<espp::detail::canopen::NmtCommand> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(espp::detail::canopen::NmtCommand cmd, FormatContext &ctx) const {
    using espp::detail::canopen::NmtCommand;
    std::string_view s = "Unknown";
    switch (cmd) {
    case NmtCommand::Start:
      s = "Start";
      break;
    case NmtCommand::Stop:
      s = "Stop";
      break;
    case NmtCommand::PreOperational:
      s = "Pre-operational";
      break;
    case NmtCommand::ResetNode:
      s = "Reset node";
      break;
    case NmtCommand::ResetCommunication:
      s = "Reset communication";
      break;
    }
    return fmt::formatter<std::string_view>::format(s, ctx);
  }
};
