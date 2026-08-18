#pragma once

// Host-buildable CANopen (CiA 301) wire core: frame builders / parsers with no
// ESP-IDF (or even espp) dependencies -- pure C++20 standard library. All
// multi-byte object data is little-endian per CiA 301. The espp::CanopenClient
// component layers transport wiring, blocking SDO transactions, and logging on
// top of this core; the host unit tests exercise this file directly.

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace espp::detail {

/// \brief A transport-agnostic classic CAN 2.0 frame.
/// \details Deliberately mirrors espp::Twai::Message field-for-field so the two
///          convert trivially, without this component depending on any CAN
///          driver.
struct CanFrame {
  uint32_t id{0};                ///< Arbitration ID (11-bit standard, or 29-bit if \c extended).
  bool extended{false};          ///< True for an extended (29-bit) ID. CANopen uses standard IDs.
  bool rtr{false};               ///< True for a Remote Transmission Request frame.
  uint8_t dlc{0};                ///< Number of valid data bytes (0-8).
  std::array<uint8_t, 8> data{}; ///< Frame payload (only the first \c dlc bytes valid).
};

namespace canopen {

/// @name Well-known COB-ID bases (pre-defined connection set, CiA 301 §7.3.5)
/// @{
inline constexpr uint32_t COB_NMT = 0x000;            ///< NMT master command (broadcast).
inline constexpr uint32_t COB_SYNC = 0x080;           ///< SYNC producer.
inline constexpr uint32_t COB_EMCY_BASE = 0x080;      ///< EMCY: 0x080 + node id.
inline constexpr uint32_t COB_TPDO1_BASE = 0x180;     ///< TPDO1: 0x180 + node id.
inline constexpr uint32_t COB_RPDO1_BASE = 0x200;     ///< RPDO1: 0x200 + node id.
inline constexpr uint32_t COB_TPDO2_BASE = 0x280;     ///< TPDO2: 0x280 + node id.
inline constexpr uint32_t COB_RPDO2_BASE = 0x300;     ///< RPDO2: 0x300 + node id.
inline constexpr uint32_t COB_TPDO3_BASE = 0x380;     ///< TPDO3: 0x380 + node id.
inline constexpr uint32_t COB_RPDO3_BASE = 0x400;     ///< RPDO3: 0x400 + node id.
inline constexpr uint32_t COB_TPDO4_BASE = 0x480;     ///< TPDO4: 0x480 + node id.
inline constexpr uint32_t COB_RPDO4_BASE = 0x500;     ///< RPDO4: 0x500 + node id.
inline constexpr uint32_t COB_SDO_TX_BASE = 0x580;    ///< SDO server->client (response).
inline constexpr uint32_t COB_SDO_RX_BASE = 0x600;    ///< SDO client->server (request).
inline constexpr uint32_t COB_HEARTBEAT_BASE = 0x700; ///< Heartbeat / boot-up: 0x700 + node id.
/// @}

/// \brief NMT master command specifiers (CiA 301 §7.2.8.3.1).
enum class NmtCommand : uint8_t {
  Start = 0x01,              ///< Start remote node (-> Operational).
  Stop = 0x02,               ///< Stop remote node (-> Stopped).
  PreOperational = 0x80,     ///< Enter pre-operational.
  ResetNode = 0x81,          ///< Reset node (application reset).
  ResetCommunication = 0x82, ///< Reset communication.
};

/// \brief NMT states as reported in heartbeat / boot-up frames (CiA 301 §7.2.8.3.2.2).
enum class NmtState : uint8_t {
  BootUp = 0x00,         ///< Boot-up message (device just initialized).
  Stopped = 0x04,        ///< Stopped.
  Operational = 0x05,    ///< Operational.
  PreOperational = 0x7F, ///< Pre-operational.
  Unknown = 0xFF,        ///< Not a standard state (or nothing heard yet).
};

/// \brief Store an unsigned value little-endian into \p out (LSB first).
/// \param value Value to store.
/// \param out Destination bytes; \p num_bytes are written.
/// \param num_bytes Number of bytes to write (1..4).
inline void put_le(uint32_t value, uint8_t *out, size_t num_bytes) {
  std::generate_n(out, num_bytes, [value, shift = 0u]() mutable {
    const auto b = static_cast<uint8_t>((value >> shift) & 0xFF);
    shift += 8;
    return b;
  });
}

/// \brief Load a little-endian unsigned value from \p in.
/// \param in Source bytes.
/// \param num_bytes Number of bytes to read (1..4).
/// \return The decoded value.
inline uint32_t get_le(const uint8_t *in, size_t num_bytes) {
  uint32_t value = 0;
  for (size_t i = 0; i < num_bytes; ++i) {
    value |= static_cast<uint32_t>(in[i]) << (8 * i);
  }
  return value;
}

/// \brief Build an NMT master command frame (COB-ID 0x000).
/// \param command The NMT command specifier.
/// \param node_id Target node id (1-127), or 0 to address all nodes.
/// \return The frame to transmit.
inline CanFrame make_nmt(NmtCommand command, uint8_t node_id) {
  CanFrame f;
  f.id = COB_NMT;
  f.dlc = 2;
  f.data[0] = static_cast<uint8_t>(command);
  f.data[1] = node_id;
  return f;
}

/// \brief Build a SYNC frame (COB-ID 0x080, no data).
/// \return The frame to transmit.
inline CanFrame make_sync() {
  CanFrame f;
  f.id = COB_SYNC;
  f.dlc = 0;
  return f;
}

/// \brief Build an RPDO (or any raw data) frame for a given COB-ID.
/// \param cob_id The COB-ID to transmit on (e.g. 0x200 + node id for RPDO1).
/// \param data Packed application data (up to 8 bytes; extra bytes ignored).
/// \return The frame to transmit.
inline CanFrame make_pdo(uint32_t cob_id, std::span<const uint8_t> data) {
  CanFrame f;
  f.id = cob_id;
  f.dlc = static_cast<uint8_t>(std::min<size_t>(data.size(), f.data.size()));
  std::copy_n(data.begin(), f.dlc, f.data.begin());
  return f;
}

/// \brief Parse a heartbeat / boot-up frame (COB-ID 0x700 + node id).
/// \param frame The received frame.
/// \param[out] node_id On success, the producing node id (1-127).
/// \return The reported NMT state, or std::nullopt if \p frame is not a heartbeat.
inline std::optional<NmtState> parse_heartbeat(const CanFrame &frame, uint8_t &node_id) {
  if (frame.extended || frame.rtr || frame.dlc < 1) {
    return std::nullopt;
  }
  if (frame.id <= COB_HEARTBEAT_BASE || frame.id > COB_HEARTBEAT_BASE + 0x7F) {
    return std::nullopt;
  }
  node_id = static_cast<uint8_t>(frame.id - COB_HEARTBEAT_BASE);
  const uint8_t state = frame.data[0] & 0x7F; // mask the (historic) toggle bit
  switch (state) {
  case 0x00:
    return NmtState::BootUp;
  case 0x04:
    return NmtState::Stopped;
  case 0x05:
    return NmtState::Operational;
  case 0x7F:
    return NmtState::PreOperational;
  default:
    return NmtState::Unknown;
  }
}

/// @name SDO (Service Data Object) protocol, expedited + segmented upload (CiA 301 §7.2.4)
/// @{

/// \brief Build an expedited SDO download (write) initiate request.
/// \details Byte 0 is the command: ccs=1 (initiate download), e=1 (expedited),
///          s=1 (size indicated), n = 4 - \p data.size() empty bytes; i.e. 0x2F
///          for 1 byte, 0x2B for 2 bytes, 0x23 for 4 bytes.
/// \param node_id Server node id (request goes to COB-ID 0x600 + node id).
/// \param index Object dictionary index.
/// \param subindex Object dictionary subindex.
/// \param data Object data, little-endian, 1, 2, or 4 bytes.
/// \return The frame to transmit.
inline CanFrame make_sdo_expedited_download(uint8_t node_id, uint16_t index, uint8_t subindex,
                                            std::span<const uint8_t> data) {
  CanFrame f;
  f.id = COB_SDO_RX_BASE + node_id;
  f.dlc = 8;
  const auto len = std::min<size_t>(data.size(), 4);
  const auto n = static_cast<uint8_t>(4 - len);
  f.data[0] = static_cast<uint8_t>(0x20 | (n << 2) | 0x02 | 0x01); // ccs=1, e=1, s=1
  put_le(index, &f.data[1], 2);
  f.data[3] = subindex;
  std::copy_n(data.begin(), len, f.data.begin() + 4);
  return f;
}

/// \brief Build an SDO upload (read) initiate request (ccs=2, byte 0 = 0x40).
/// \param node_id Server node id.
/// \param index Object dictionary index.
/// \param subindex Object dictionary subindex.
/// \return The frame to transmit.
inline CanFrame make_sdo_upload_request(uint8_t node_id, uint16_t index, uint8_t subindex) {
  CanFrame f;
  f.id = COB_SDO_RX_BASE + node_id;
  f.dlc = 8;
  f.data[0] = 0x40; // ccs=2 (initiate upload)
  put_le(index, &f.data[1], 2);
  f.data[3] = subindex;
  return f;
}

/// \brief Build an SDO upload segment request (ccs=3, byte 0 = 0x60 | toggle<<4).
/// \param node_id Server node id.
/// \param toggle Toggle bit; must alternate starting at false for the first segment.
/// \return The frame to transmit.
inline CanFrame make_sdo_upload_segment_request(uint8_t node_id, bool toggle) {
  CanFrame f;
  f.id = COB_SDO_RX_BASE + node_id;
  f.dlc = 8;
  f.data[0] = static_cast<uint8_t>(0x60 | (toggle ? 0x10 : 0x00));
  return f;
}

/// \brief Build an SDO abort transfer frame (cs=0x80).
/// \param node_id Server node id (the abort is sent on the request COB-ID).
/// \param index Object dictionary index the abort refers to.
/// \param subindex Object dictionary subindex the abort refers to.
/// \param abort_code CiA 301 abort code.
/// \return The frame to transmit.
inline CanFrame make_sdo_abort(uint8_t node_id, uint16_t index, uint8_t subindex,
                               uint32_t abort_code) {
  CanFrame f;
  f.id = COB_SDO_RX_BASE + node_id;
  f.dlc = 8;
  f.data[0] = 0x80;
  put_le(index, &f.data[1], 2);
  f.data[3] = subindex;
  put_le(abort_code, &f.data[4], 4);
  return f;
}

/// \brief A parsed SDO server response (COB-ID 0x580 + node id).
struct SdoResponse {
  /// \brief The kind of response, from the server command specifier (scs).
  enum class Type : uint8_t {
    DownloadOk,          ///< scs=3: download initiate confirmed.
    ExpeditedUpload,     ///< scs=2, e=1: expedited upload data in \c data.
    SegmentedUploadInit, ///< scs=2, e=0: segmented upload; \c total_size may be indicated.
    UploadSegment,       ///< scs=0: upload segment data in \c data.
    Abort,               ///< cs=0x80: transfer aborted; see \c abort_code.
    Unknown,             ///< Unrecognized command specifier.
  };
  Type type{Type::Unknown};
  uint16_t index{0};             ///< Object index (valid for initiate / abort responses).
  uint8_t subindex{0};           ///< Object subindex (valid for initiate / abort responses).
  std::array<uint8_t, 7> data{}; ///< Payload bytes (expedited: up to 4; segment: up to 7).
  uint8_t len{0};                ///< Number of valid bytes in \c data.
  bool size_indicated{false};    ///< True if the server indicated a size.
  uint32_t total_size{0};        ///< Total transfer size (SegmentedUploadInit with size_indicated).
  bool toggle{false};            ///< Toggle bit (UploadSegment).
  bool last{false};              ///< True if this UploadSegment is the final one (c bit).
  uint32_t abort_code{0};        ///< Abort code (Abort).
};

/// \brief Parse an SDO server response frame.
/// \param frame A frame received on COB-ID 0x580 + node id (caller checks the id).
/// \return The parsed response; type is Type::Unknown if the command specifier
///         is unrecognized or the frame is malformed.
inline SdoResponse parse_sdo_response(const CanFrame &frame) {
  SdoResponse r;
  if (frame.dlc < 1) {
    return r;
  }
  const uint8_t cmd = frame.data[0];
  const uint8_t scs = cmd >> 5;
  if (cmd == 0x80) {
    // abort transfer
    r.type = SdoResponse::Type::Abort;
    r.index = static_cast<uint16_t>(get_le(&frame.data[1], 2));
    r.subindex = frame.data[3];
    r.abort_code = get_le(&frame.data[4], 4);
    return r;
  }
  switch (scs) {
  case 3: // initiate download response
    r.type = SdoResponse::Type::DownloadOk;
    r.index = static_cast<uint16_t>(get_le(&frame.data[1], 2));
    r.subindex = frame.data[3];
    return r;
  case 2: { // initiate upload response
    r.index = static_cast<uint16_t>(get_le(&frame.data[1], 2));
    r.subindex = frame.data[3];
    const bool expedited = (cmd & 0x02) != 0;
    r.size_indicated = (cmd & 0x01) != 0;
    if (expedited) {
      r.type = SdoResponse::Type::ExpeditedUpload;
      const auto n = static_cast<uint8_t>((cmd >> 2) & 0x03);
      r.len = r.size_indicated ? static_cast<uint8_t>(4 - n) : 4;
      std::copy_n(frame.data.begin() + 4, r.len, r.data.begin());
    } else {
      r.type = SdoResponse::Type::SegmentedUploadInit;
      if (r.size_indicated) {
        r.total_size = get_le(&frame.data[4], 4);
      }
    }
    return r;
  }
  case 0: { // upload segment response
    r.type = SdoResponse::Type::UploadSegment;
    r.toggle = (cmd & 0x10) != 0;
    r.last = (cmd & 0x01) != 0;
    const auto n = static_cast<uint8_t>((cmd >> 1) & 0x07); // bytes NOT containing data
    r.len = static_cast<uint8_t>(7 - n);
    std::copy_n(frame.data.begin() + 1, r.len, r.data.begin());
    return r;
  }
  default:
    return r;
  }
}

/// \brief Small accumulator for a segmented SDO upload.
/// \details Feed the parsed SegmentedUploadInit response, then each parsed
///          UploadSegment response; verifies the toggle-bit alternation and
///          collects the payload bytes. The caller drives the request side
///          (make_sdo_upload_segment_request with next_toggle()).
class SdoSegmentedUpload {
public:
  /// \brief Start a transfer from a parsed SegmentedUploadInit response.
  /// \param init The parsed initiate response.
  void start(const SdoResponse &init) {
    data_.clear();
    toggle_ = false;
    done_ = false;
    if (init.size_indicated) {
      data_.reserve(init.total_size);
    }
  }

  /// \brief The toggle bit to use for the next segment request.
  bool next_toggle() const { return toggle_; }

  /// \brief Consume a parsed UploadSegment response.
  /// \param segment The parsed segment response.
  /// \return False on toggle-bit mismatch (protocol error), true otherwise.
  bool consume(const SdoResponse &segment) {
    if (segment.toggle != toggle_) {
      return false;
    }
    data_.append(reinterpret_cast<const char *>(segment.data.data()), segment.len);
    toggle_ = !toggle_;
    done_ = segment.last;
    return true;
  }

  /// \brief True once the final segment (c bit) has been consumed.
  bool done() const { return done_; }

  /// \brief The accumulated payload bytes.
  const std::string &data() const { return data_; }

private:
  std::string data_{};
  bool toggle_{false};
  bool done_{false};
};

/// \brief Map a CiA 301 SDO abort code to a human-readable string.
/// \param abort_code The 32-bit abort code from an SDO abort frame.
/// \return A static description string ("unknown abort code" if unmapped).
inline const char *sdo_abort_to_string(uint32_t abort_code) {
  switch (abort_code) {
  case 0x05030000:
    return "toggle bit not alternated";
  case 0x05040000:
    return "SDO protocol timed out";
  case 0x05040001:
    return "invalid or unknown command specifier";
  case 0x05040002:
    return "invalid block size";
  case 0x05040003:
    return "invalid sequence number";
  case 0x05040004:
    return "CRC error";
  case 0x05040005:
    return "out of memory";
  case 0x06010000:
    return "unsupported access to object";
  case 0x06010001:
    return "attempt to read a write-only object";
  case 0x06010002:
    return "attempt to write a read-only object";
  case 0x06020000:
    return "object does not exist in the object dictionary";
  case 0x06040041:
    return "object cannot be mapped to the PDO";
  case 0x06040042:
    return "number and length of mapped objects would exceed PDO length";
  case 0x06040043:
    return "general parameter incompatibility";
  case 0x06040047:
    return "general internal incompatibility in the device";
  case 0x06060000:
    return "access failed due to a hardware error";
  case 0x06070010:
    return "data type does not match, length of service parameter does not match";
  case 0x06070012:
    return "data type does not match, length of service parameter too high";
  case 0x06070013:
    return "data type does not match, length of service parameter too low";
  case 0x06090011:
    return "subindex does not exist";
  case 0x06090030:
    return "invalid value for parameter";
  case 0x06090031:
    return "value of parameter written too high";
  case 0x06090032:
    return "value of parameter written too low";
  case 0x06090036:
    return "maximum value is less than minimum value";
  case 0x08000000:
    return "general error";
  case 0x08000020:
    return "data cannot be transferred or stored to the application";
  case 0x08000021:
    return "data cannot be transferred or stored because of local control";
  case 0x08000022:
    return "data cannot be transferred or stored because of the present device state";
  case 0x08000023:
    return "object dictionary dynamic generation fails or no object dictionary present";
  default:
    return "unknown abort code";
  }
}

/// @}

} // namespace canopen

namespace ds402 {

/// @name Standard CiA 402 / device object dictionary indices
/// @{
inline constexpr uint16_t OBJ_DEVICE_TYPE = 0x1000;        ///< Device type (u32).
inline constexpr uint16_t OBJ_ERROR_REGISTER = 0x1001;     ///< Error register (u8).
inline constexpr uint16_t OBJ_DEVICE_NAME = 0x1008;        ///< Manufacturer device name (string).
inline constexpr uint16_t OBJ_IDENTITY = 0x1018;           ///< Identity object (subs 1-4).
inline constexpr uint16_t OBJ_CONTROLWORD = 0x6040;        ///< Controlword (u16).
inline constexpr uint16_t OBJ_STATUSWORD = 0x6041;         ///< Statusword (u16).
inline constexpr uint16_t OBJ_MODES_OF_OPERATION = 0x6060; ///< Modes of operation (i8).
inline constexpr uint16_t OBJ_MODES_OF_OPERATION_DISPLAY = 0x6061; ///< Modes display (i8).
inline constexpr uint16_t OBJ_POSITION_ACTUAL = 0x6064;            ///< Position actual value (i32).
inline constexpr uint16_t OBJ_VELOCITY_ACTUAL = 0x606C;            ///< Velocity actual value (i32).
inline constexpr uint16_t OBJ_TARGET_POSITION = 0x607A;            ///< Target position (i32).
inline constexpr uint16_t OBJ_PROFILE_VELOCITY = 0x6081;           ///< Profile velocity (u32).
inline constexpr uint16_t OBJ_PROFILE_ACCELERATION = 0x6083;       ///< Profile acceleration (u32).
inline constexpr uint16_t OBJ_PROFILE_DECELERATION = 0x6084;       ///< Profile deceleration (u32).
inline constexpr uint16_t OBJ_TARGET_VELOCITY = 0x60FF;            ///< Target velocity (i32).
/// @}

/// @name Controlword command values (CiA 402 §8.2.1)
/// @{
inline constexpr uint16_t CW_SHUTDOWN = 0x0006;         ///< Shutdown -> Ready to switch on.
inline constexpr uint16_t CW_SWITCH_ON = 0x0007;        ///< Switch on -> Switched on.
inline constexpr uint16_t CW_ENABLE_OPERATION = 0x000F; ///< Enable operation.
inline constexpr uint16_t CW_DISABLE_VOLTAGE = 0x0000;  ///< Disable voltage -> Switch on disabled.
inline constexpr uint16_t CW_QUICK_STOP = 0x0002;       ///< Quick stop.
inline constexpr uint16_t CW_FAULT_RESET = 0x0080;      ///< Fault reset (rising edge of bit 7).
inline constexpr uint16_t CW_BIT_NEW_SETPOINT =
    0x0010; ///< Bit 4: new set-point (profile position).
inline constexpr uint16_t CW_BIT_CHANGE_SET_IMMEDIATELY =
    0x0020;                                         ///< Bit 5: change set immediately.
inline constexpr uint16_t CW_BIT_RELATIVE = 0x0040; ///< Bit 6: target position is relative.
/// @}

/// @name Statusword bits (CiA 402 §8.2.2)
/// @{
inline constexpr uint16_t SW_BIT_TARGET_REACHED = 0x0400;       ///< Bit 10: target reached.
inline constexpr uint16_t SW_BIT_SETPOINT_ACKNOWLEDGE = 0x1000; ///< Bit 12 (profile position).
/// @}

/// \brief CiA 402 power drive system state, decoded from the statusword.
enum class State : uint8_t {
  NotReadyToSwitchOn,  ///< xxxx xxxx x0xx 0000
  SwitchOnDisabled,    ///< xxxx xxxx x1xx 0000
  ReadyToSwitchOn,     ///< xxxx xxxx x01x 0001
  SwitchedOn,          ///< xxxx xxxx x01x 0011
  OperationEnabled,    ///< xxxx xxxx x01x 0111
  QuickStopActive,     ///< xxxx xxxx x00x 0111
  FaultReactionActive, ///< xxxx xxxx x0xx 1111
  Fault,               ///< xxxx xxxx x0xx 1000
  Unknown,             ///< Statusword did not match any standard state pattern.
};

/// \brief Standard CiA 402 modes of operation (object 0x6060 / 0x6061).
enum class OperatingMode : int8_t {
  ProfilePosition = 1, ///< Profile position mode (pp).
  ProfileVelocity = 3, ///< Profile velocity mode (pv).
  ProfileTorque = 4,   ///< Profile torque mode (tq).
  Homing = 6,          ///< Homing mode (hm).
};

/// \brief Decode a CiA 402 statusword into a drive state.
/// \details Applies the standard bit masks: mask 0x4F distinguishes
///          Not-ready / Switch-on-disabled / Fault / Fault-reaction; mask 0x6F
///          distinguishes Ready-to-switch-on / Switched-on / Operation-enabled /
///          Quick-stop-active.
/// \param statusword The raw statusword (object 0x6041).
/// \return The decoded state.
inline State decode_state(uint16_t statusword) {
  switch (statusword & 0x4F) {
  case 0x00:
    return State::NotReadyToSwitchOn;
  case 0x40:
    return State::SwitchOnDisabled;
  case 0x08:
    return State::Fault;
  case 0x0F:
    return State::FaultReactionActive;
  default:
    break;
  }
  switch (statusword & 0x6F) {
  case 0x21:
    return State::ReadyToSwitchOn;
  case 0x23:
    return State::SwitchedOn;
  case 0x27:
    return State::OperationEnabled;
  case 0x07:
    return State::QuickStopActive;
  default:
    return State::Unknown;
  }
}

/// \brief Get a human-readable name for a CiA 402 drive state.
/// \param state The decoded state.
/// \return A static name string.
inline const char *state_to_string(State state) {
  switch (state) {
  case State::NotReadyToSwitchOn:
    return "Not ready to switch on";
  case State::SwitchOnDisabled:
    return "Switch on disabled";
  case State::ReadyToSwitchOn:
    return "Ready to switch on";
  case State::SwitchedOn:
    return "Switched on";
  case State::OperationEnabled:
    return "Operation enabled";
  case State::QuickStopActive:
    return "Quick stop active";
  case State::FaultReactionActive:
    return "Fault reaction active";
  case State::Fault:
    return "Fault";
  default:
    return "Unknown";
  }
}

} // namespace ds402

} // namespace espp::detail
