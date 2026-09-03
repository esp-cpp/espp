#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <unordered_map>

#include "base_component.hpp"
#include "canopen_format_helpers.hpp"
#include "detail/canopen_core.hpp"

namespace espp {

/// \brief A lightweight CANopen (CiA 301) client / master for a single server node.
/// \details Implements the master-side services needed to drive a typical
///          CANopen device (e.g. a Basicmicro MCP236/MCP266 motor controller):
///          - NMT master commands (start / stop / pre-operational / reset)
///          - Heartbeat & boot-up consumption (cached state + optional callback)
///          - SDO client: expedited upload/download of 1/2/4-byte objects, and
///            segmented upload (for strings such as device name 0x1008)
///          - PDO helpers: RPDO transmit and per-COB-ID TPDO reception dispatch
///          - SYNC transmission
///
///          The client is transport-agnostic: it transmits by calling the
///          configured \c send function with a espp::detail::CanFrame, and the
///          application feeds every received frame to process_frame() (e.g.
///          from the espp::Twai \c on_receive callback -- the frame layouts
///          match field-for-field).
///
///          SDO transactions are blocking: the calling task sends the request
///          and waits on a condition variable until process_frame() delivers
///          the matching response (COB-ID 0x580 + node id) or the configured
///          timeout expires. \b Note: process_frame() must therefore be called
///          from a different task than the one performing SDO reads/writes;
///          with espp::Twai this is automatically the case since \c on_receive
///          runs in the Twai receive task. One SDO transaction may be in
///          flight per client at a time (serialized internally by a mutex);
///          NMT / SYNC / PDO helpers are non-blocking and unserialized.
///
/// \section canopen_ex0 CANopen Client Example
/// \snippet canopen_example.cpp canopen example
class CanopenClient : public BaseComponent {
public:
  using CanFrame = detail::CanFrame;              ///< Transport-agnostic CAN frame type.
  using NmtCommand = detail::canopen::NmtCommand; ///< NMT master command specifier.
  using NmtState = detail::canopen::NmtState;     ///< NMT state (heartbeat / boot-up).

  /// \brief Function used to transmit a frame on the bus.
  /// \details Should return true if the frame was (queued to be) sent.
  typedef std::function<bool(const CanFrame &frame)> send_fn;

  /// \brief Callback invoked (from the process_frame() context) for every
  ///        heartbeat / boot-up frame received from any node.
  typedef std::function<void(uint8_t node_id, NmtState state)> heartbeat_callback_fn;

  /// \brief Callback invoked (from the process_frame() context) for a received
  ///        frame on a registered TPDO COB-ID.
  typedef std::function<void(const CanFrame &frame)> pdo_callback_fn;

  /// \brief Configuration for the CanopenClient.
  struct Config {
    uint8_t node_id;                             ///< Server node id (1-127) this client talks to.
    send_fn send;                                ///< Function used to transmit frames on the bus.
    std::chrono::milliseconds sdo_timeout{100};  ///< Timeout for one SDO round-trip.
    heartbeat_callback_fn on_heartbeat{nullptr}; ///< Optional heartbeat / boot-up callback.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /// \brief Create a CANopen client.
  /// \param config The configuration.
  explicit CanopenClient(const Config &config)
      : BaseComponent("CanopenClient", config.log_level)
      , node_id_(config.node_id)
      , send_(config.send)
      , sdo_timeout_(config.sdo_timeout)
      , on_heartbeat_(config.on_heartbeat) {
    // A CANopen node id is 1-127; 0 is the broadcast/unconfigured value and would
    // make SDO addressing (0x580/0x600 + id) and heartbeat matching wrong.
    if (node_id_ < 1 || node_id_ > 127) {
      logger_.error("node_id {} is out of range (1-127); clamping to 1 — set a valid node id",
                    node_id_);
      node_id_ = 1;
    }
  }

  /// \brief The configured server node id.
  uint8_t node_id() const { return node_id_; }

  /// \brief Feed a received CAN frame to the client.
  /// \details Call this for every frame received from the bus (e.g. from the
  ///          espp::Twai \c on_receive callback). Dispatches SDO responses to
  ///          the waiting transaction, caches heartbeat states (invoking the
  ///          optional heartbeat callback), and dispatches registered TPDO
  ///          callbacks. Must not be called from the task performing SDO
  ///          transactions (see class description).
  /// \param frame The received frame.
  void process_frame(const CanFrame &frame) {
    // SDO response from our server node?
    if (frame.id == detail::canopen::COB_SDO_TX_BASE + node_id_) {
      using Type = detail::canopen::SdoResponse::Type;
      const auto parsed = detail::canopen::parse_sdo_response(frame);
      bool delivered = false;
      {
        std::lock_guard<std::mutex> lock(response_mutex_);
        if (awaiting_response_) {
          // Only deliver a response that belongs to the in-flight request: a
          // late response from a previously timed-out transaction (or any
          // unrelated server traffic) must not complete the wrong one.
          // Upload-segment responses carry no index/subindex, so they match by
          // expected phase; every other response type echoes the object
          // address and must match it. Malformed frames (Unknown) never match.
          const bool is_segment = parsed.type == Type::UploadSegment;
          bool matches = false;
          if (expected_segment_) {
            matches = is_segment;
          } else {
            matches = !is_segment && parsed.type != Type::Unknown &&
                      parsed.index == expected_index_ && parsed.subindex == expected_subindex_;
          }
          if (matches) {
            response_ = parsed;
            awaiting_response_ = false;
            delivered = true;
          }
        }
      }
      if (delivered) {
        response_cv_.notify_all();
      } else {
        logger_.warn("Ignoring stale/unexpected SDO response (type {}, 0x{:04X}:{:02X})",
                     static_cast<int>(parsed.type), parsed.index, parsed.subindex);
      }
      return;
    }
    // heartbeat / boot-up from any node?
    uint8_t hb_node = 0;
    if (auto state = detail::canopen::parse_heartbeat(frame, hb_node); state.has_value()) {
      logger_.debug("Heartbeat from node {}: state 0x{:02X}", hb_node,
                    static_cast<uint8_t>(frame.data[0]));
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        node_states_[hb_node] = *state;
      }
      if (on_heartbeat_) {
        on_heartbeat_(hb_node, *state);
      }
      return;
    }
    // registered TPDO?
    pdo_callback_fn callback{nullptr};
    {
      std::lock_guard<std::mutex> lock(pdo_mutex_);
      if (auto it = pdo_callbacks_.find(frame.id); it != pdo_callbacks_.end()) {
        callback = it->second;
      }
    }
    if (callback) {
      callback(frame);
    }
  }

  /// @name NMT master / SYNC / PDO (non-blocking)
  /// @{

  /// \brief Send an NMT master command.
  /// \param command The command specifier.
  /// \param target_node_id Target node id, or 0 to address all nodes.
  /// \param ec Set on transmit failure.
  /// \return True on success.
  bool send_nmt(NmtCommand command, uint8_t target_node_id, std::error_code &ec) {
    return send_frame(detail::canopen::make_nmt(command, target_node_id), ec);
  }

  /// \brief NMT-start the configured server node. \param ec Set on failure. \return True on
  /// success.
  bool nmt_start(std::error_code &ec) { return send_nmt(NmtCommand::Start, node_id_, ec); }
  /// \brief NMT-stop the configured server node. \param ec Set on failure. \return True on success.
  bool nmt_stop(std::error_code &ec) { return send_nmt(NmtCommand::Stop, node_id_, ec); }
  /// \brief Put the configured server node into pre-operational. \param ec Set on failure.
  /// \return True on success.
  bool nmt_pre_operational(std::error_code &ec) {
    return send_nmt(NmtCommand::PreOperational, node_id_, ec);
  }
  /// \brief Reset the configured server node (application reset). \param ec Set on failure.
  /// \return True on success.
  bool nmt_reset_node(std::error_code &ec) { return send_nmt(NmtCommand::ResetNode, node_id_, ec); }
  /// \brief Reset communication of the configured server node. \param ec Set on failure.
  /// \return True on success.
  bool nmt_reset_communication(std::error_code &ec) {
    return send_nmt(NmtCommand::ResetCommunication, node_id_, ec);
  }

  /// \brief Send a SYNC frame (COB-ID 0x080).
  /// \param ec Set on transmit failure.
  /// \return True on success.
  bool send_sync(std::error_code &ec) { return send_frame(detail::canopen::make_sync(), ec); }

  /// \brief Transmit an RPDO (build + send a data frame on \p cob_id).
  /// \param cob_id COB-ID to transmit on (e.g. 0x200 + node id for RPDO1).
  /// \param data Packed application data (up to 8 bytes).
  /// \param ec Set on transmit failure.
  /// \return True on success.
  bool send_rpdo(uint32_t cob_id, std::span<const uint8_t> data, std::error_code &ec) {
    return send_frame(detail::canopen::make_pdo(cob_id, data), ec);
  }

  /// \brief Register a callback for received frames on a TPDO COB-ID.
  /// \param cob_id COB-ID to match (e.g. 0x180 + node id for TPDO1).
  /// \param callback Invoked from the process_frame() context; replaces any
  ///        previous callback for this COB-ID.
  void register_tpdo_callback(uint32_t cob_id, pdo_callback_fn callback) {
    std::lock_guard<std::mutex> lock(pdo_mutex_);
    pdo_callbacks_[cob_id] = callback;
  }

  /// \brief Remove the callback registered for a TPDO COB-ID.
  /// \param cob_id The COB-ID whose callback should be removed.
  void unregister_tpdo_callback(uint32_t cob_id) {
    std::lock_guard<std::mutex> lock(pdo_mutex_);
    pdo_callbacks_.erase(cob_id);
  }

  /// @}

  /// @name Heartbeat state
  /// @{

  /// \brief The last NMT state heard (via heartbeat / boot-up) from a node.
  /// \param target_node_id The node id to query.
  /// \return The last state, or std::nullopt if nothing was heard from that node.
  std::optional<NmtState> get_nmt_state(uint8_t target_node_id) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (auto it = node_states_.find(target_node_id); it != node_states_.end()) {
      return it->second;
    }
    return std::nullopt;
  }

  /// \brief The last NMT state heard from the configured server node.
  /// \return The last state, or std::nullopt if nothing was heard yet.
  std::optional<NmtState> get_nmt_state() const { return get_nmt_state(node_id_); }

  /// @}

  /// @name SDO client (blocking, expedited)
  /// @{

  /// \brief Write (SDO expedited download) raw little-endian object data.
  /// \param index Object dictionary index.
  /// \param subindex Object dictionary subindex.
  /// \param data Object data, little-endian, 1, 2, or 4 bytes.
  /// \param ec Set on transmit failure, timeout, or SDO abort.
  /// \return True on success.
  bool sdo_download(uint16_t index, uint8_t subindex, std::span<const uint8_t> data,
                    std::error_code &ec) {
    // Expedited transfers are only defined for 1, 2 or 4 bytes (CiA 301);
    // anything else would silently encode a wrong size on the wire.
    if (data.size() != 1 && data.size() != 2 && data.size() != 4) {
      logger_.error("SDO download 0x{:04X}:{:02X}: invalid expedited size {} (must be 1, 2 or 4)",
                    index, subindex, data.size());
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    std::lock_guard<std::mutex> lock(sdo_mutex_);
    detail::canopen::SdoResponse response;
    if (!sdo_transact(detail::canopen::make_sdo_expedited_download(node_id_, index, subindex, data),
                      response, index, subindex, ec)) {
      return false;
    }
    if (response.type != detail::canopen::SdoResponse::Type::DownloadOk) {
      logger_.error("SDO download 0x{:04X}:{:02X}: unexpected response type", index, subindex);
      ec = std::make_error_code(std::errc::protocol_error);
      return false;
    }
    return true;
  }

  /// \brief Read (SDO expedited upload) raw little-endian object data.
  /// \param index Object dictionary index.
  /// \param subindex Object dictionary subindex.
  /// \param out Destination for the object data (little-endian).
  /// \param ec Set on transmit failure, timeout, SDO abort, or if the object is
  ///        larger than \p out (use read_string() for segmented transfers).
  /// \return Number of bytes read (> 0), or 0 on error.
  size_t sdo_upload(uint16_t index, uint8_t subindex, std::span<uint8_t> out, std::error_code &ec) {
    std::lock_guard<std::mutex> lock(sdo_mutex_);
    detail::canopen::SdoResponse response;
    if (!sdo_transact(detail::canopen::make_sdo_upload_request(node_id_, index, subindex), response,
                      index, subindex, ec)) {
      return 0;
    }
    if (response.type != detail::canopen::SdoResponse::Type::ExpeditedUpload) {
      logger_.error("SDO upload 0x{:04X}:{:02X}: not an expedited response", index, subindex);
      ec = std::make_error_code(std::errc::protocol_error);
      return 0;
    }
    // When the server INDICATED a size, the object must fit the caller's buffer
    // (a larger object is a real width mismatch -> error below). When it did NOT
    // indicate a size, CiA 301 says all four expedited data bytes are valid and
    // the caller's requested width governs, so take the low out.size() bytes —
    // otherwise a conformant u8/u16 read against a server that leaves the size
    // bit clear (where the core reports len == 4) would spuriously fail.
    const size_t n =
        (response.size_indicated || response.len <= out.size()) ? response.len : out.size();
    if (n > out.size()) {
      logger_.error(
          "SDO upload 0x{:04X}:{:02X}: object is {} bytes, larger than the {}-byte buffer", index,
          subindex, response.len, out.size());
      ec = std::make_error_code(std::errc::protocol_error);
      return 0;
    }
    std::copy_n(response.data.begin(), n, out.begin());
    return n;
  }

  /// \brief Read a string object via SDO segmented (or expedited) upload.
  /// \details Handles the toggle-bit protocol for multi-segment transfers; used
  ///          e.g. for the manufacturer device name (0x1008).
  /// \param index Object dictionary index.
  /// \param subindex Object dictionary subindex.
  /// \param ec Set on transmit failure, timeout, SDO abort, or toggle error.
  /// \return The string data, or an empty string on error.
  std::string read_string(uint16_t index, uint8_t subindex, std::error_code &ec) {
    std::lock_guard<std::mutex> lock(sdo_mutex_);
    detail::canopen::SdoResponse response;
    if (!sdo_transact(detail::canopen::make_sdo_upload_request(node_id_, index, subindex), response,
                      index, subindex, ec)) {
      return {};
    }
    using Type = detail::canopen::SdoResponse::Type;
    if (response.type == Type::ExpeditedUpload) {
      return {reinterpret_cast<const char *>(response.data.data()), response.len};
    }
    if (response.type != Type::SegmentedUploadInit) {
      logger_.error("SDO upload 0x{:04X}:{:02X}: unexpected response type", index, subindex);
      ec = std::make_error_code(std::errc::protocol_error);
      return {};
    }
    detail::canopen::SdoSegmentedUpload assembler;
    if (!assembler.start(response)) {
      // The remote-indicated total size exceeds the assembler's cap
      // (remote-supplied, so never trusted for an unbounded reservation).
      logger_.error("SDO segmented upload 0x{:04X}:{:02X}: indicated size {} exceeds the {} byte "
                    "cap; aborting transfer",
                    index, subindex, response.total_size,
                    detail::canopen::SdoSegmentedUpload::kDefaultMaxSize);
      send_frame_quietly(
          detail::canopen::make_sdo_abort(node_id_, index, subindex, 0x05040005)); // out of memory
      ec = std::make_error_code(std::errc::message_size);
      return {};
    }
    while (!assembler.done()) {
      if (!sdo_transact(
              detail::canopen::make_sdo_upload_segment_request(node_id_, assembler.next_toggle()),
              response, index, subindex, ec, /*expect_segment=*/true)) {
        return {};
      }
      if (response.type != Type::UploadSegment || !assembler.consume(response)) {
        logger_.error("SDO segmented upload 0x{:04X}:{:02X}: bad segment (toggle mismatch?)", index,
                      subindex);
        send_frame_quietly(detail::canopen::make_sdo_abort(node_id_, index, subindex, 0x05030000));
        ec = std::make_error_code(std::errc::protocol_error);
        return {};
      }
    }
    // strings may be null-padded; trim at the first NUL
    std::string data = assembler.data();
    if (auto pos = data.find('\0'); pos != std::string::npos) {
      data.resize(pos);
    }
    return data;
  }

  /// \brief Write an unsigned 8-bit object. \param index Object index. \param subindex Object
  /// subindex. \param value Value to write. \param ec Set on failure. \return True on success.
  bool write_u8(uint16_t index, uint8_t subindex, uint8_t value, std::error_code &ec) {
    return write_le(index, subindex, value, 1, ec);
  }
  /// \brief Write an unsigned 16-bit object. \param index Object index. \param subindex Object
  /// subindex. \param value Value to write. \param ec Set on failure. \return True on success.
  bool write_u16(uint16_t index, uint8_t subindex, uint16_t value, std::error_code &ec) {
    return write_le(index, subindex, value, 2, ec);
  }
  /// \brief Write an unsigned 32-bit object. \param index Object index. \param subindex Object
  /// subindex. \param value Value to write. \param ec Set on failure. \return True on success.
  bool write_u32(uint16_t index, uint8_t subindex, uint32_t value, std::error_code &ec) {
    return write_le(index, subindex, value, 4, ec);
  }
  /// \brief Write a signed 8-bit object. \param index Object index. \param subindex Object
  /// subindex. \param value Value to write. \param ec Set on failure. \return True on success.
  bool write_i8(uint16_t index, uint8_t subindex, int8_t value, std::error_code &ec) {
    return write_le(index, subindex, static_cast<uint8_t>(value), 1, ec);
  }
  /// \brief Write a signed 16-bit object. \param index Object index. \param subindex Object
  /// subindex. \param value Value to write. \param ec Set on failure. \return True on success.
  bool write_i16(uint16_t index, uint8_t subindex, int16_t value, std::error_code &ec) {
    return write_le(index, subindex, static_cast<uint16_t>(value), 2, ec);
  }
  /// \brief Write a signed 32-bit object. \param index Object index. \param subindex Object
  /// subindex. \param value Value to write. \param ec Set on failure. \return True on success.
  bool write_i32(uint16_t index, uint8_t subindex, int32_t value, std::error_code &ec) {
    return write_le(index, subindex, static_cast<uint32_t>(value), 4, ec);
  }

  /// \brief Read an unsigned 8-bit object. \param index Object index. \param subindex Object
  /// subindex. \param ec Set on failure. \return The value (0 on error).
  uint8_t read_u8(uint16_t index, uint8_t subindex, std::error_code &ec) {
    return static_cast<uint8_t>(read_le(index, subindex, 1, ec));
  }
  /// \brief Read an unsigned 16-bit object. \param index Object index. \param subindex Object
  /// subindex. \param ec Set on failure. \return The value (0 on error).
  uint16_t read_u16(uint16_t index, uint8_t subindex, std::error_code &ec) {
    return static_cast<uint16_t>(read_le(index, subindex, 2, ec));
  }
  /// \brief Read an unsigned 32-bit object. \param index Object index. \param subindex Object
  /// subindex. \param ec Set on failure. \return The value (0 on error).
  uint32_t read_u32(uint16_t index, uint8_t subindex, std::error_code &ec) {
    return read_le(index, subindex, 4, ec);
  }
  /// \brief Read a signed 8-bit object. \param index Object index. \param subindex Object
  /// subindex. \param ec Set on failure. \return The value (0 on error).
  int8_t read_i8(uint16_t index, uint8_t subindex, std::error_code &ec) {
    return static_cast<int8_t>(read_le(index, subindex, 1, ec));
  }
  /// \brief Read a signed 16-bit object. \param index Object index. \param subindex Object
  /// subindex. \param ec Set on failure. \return The value (0 on error).
  int16_t read_i16(uint16_t index, uint8_t subindex, std::error_code &ec) {
    return static_cast<int16_t>(read_le(index, subindex, 2, ec));
  }
  /// \brief Read a signed 32-bit object. \param index Object index. \param subindex Object
  /// subindex. \param ec Set on failure. \return The value (0 on error).
  int32_t read_i32(uint16_t index, uint8_t subindex, std::error_code &ec) {
    return static_cast<int32_t>(read_le(index, subindex, 4, ec));
  }

  /// \brief The abort code from the most recent SDO abort response (0 if none).
  uint32_t last_abort_code() const {
    std::lock_guard<std::mutex> lock(response_mutex_);
    return last_abort_code_;
  }

  /// \brief Human-readable description of a CiA 301 SDO abort code (e.g. for
  ///        logging the reason behind an SDO failure / last_abort_code()).
  static const char *abort_code_to_string(uint32_t abort_code) {
    return detail::canopen::sdo_abort_to_string(abort_code);
  }

  /// @}

protected:
  bool send_frame(const CanFrame &frame, std::error_code &ec) {
    if (!send_ || !send_(frame)) {
      logger_.error("Failed to send frame with id 0x{:03X}", frame.id);
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    return true;
  }

  void send_frame_quietly(const CanFrame &frame) {
    if (send_) {
      send_(frame);
    }
  }

  /// Perform one SDO request/response round-trip. Must be called with
  /// sdo_mutex_ held. On an abort response, logs the decoded abort reason and
  /// fails with protocol_error.
  bool sdo_transact(const CanFrame &request, detail::canopen::SdoResponse &response, uint16_t index,
                    uint8_t subindex, std::error_code &ec, bool expect_segment = false) {
    {
      std::lock_guard<std::mutex> lock(response_mutex_);
      awaiting_response_ = true;
      // Clear any abort code cached by a previous transaction so last_abort_code()
      // never reports a stale code from an earlier, unrelated failure.
      last_abort_code_ = 0;
      // Record what the in-flight request is for, so process_frame() can
      // reject stale/unrelated responses instead of completing the wrong
      // transaction (segment responses carry no index/subindex and are
      // matched by phase instead).
      expected_index_ = index;
      expected_subindex_ = subindex;
      expected_segment_ = expect_segment;
    }
    if (!send_frame(request, ec)) {
      std::lock_guard<std::mutex> lock(response_mutex_);
      awaiting_response_ = false;
      return false;
    }
    std::unique_lock<std::mutex> lock(response_mutex_);
    if (!response_cv_.wait_for(lock, sdo_timeout_, [this] { return !awaiting_response_; })) {
      awaiting_response_ = false;
      logger_.error("SDO 0x{:04X}:{:02X}: timed out after {} ms waiting for response from node {}",
                    index, subindex, sdo_timeout_.count(), node_id_);
      ec = std::make_error_code(std::errc::timed_out);
      return false;
    }
    response = response_;
    if (response.type == detail::canopen::SdoResponse::Type::Abort) {
      last_abort_code_ = response.abort_code;
      lock.unlock();
      logger_.error("SDO 0x{:04X}:{:02X}: aborted with code 0x{:08X} ({})", index, subindex,
                    response.abort_code, detail::canopen::sdo_abort_to_string(response.abort_code));
      ec = std::make_error_code(std::errc::protocol_error);
      return false;
    }
    return true;
  }

  bool write_le(uint16_t index, uint8_t subindex, uint32_t value, size_t num_bytes,
                std::error_code &ec) {
    std::array<uint8_t, 4> data{};
    detail::canopen::put_le(value, data.data(), num_bytes);
    return sdo_download(index, subindex, std::span<const uint8_t>(data.data(), num_bytes), ec);
  }

  uint32_t read_le(uint16_t index, uint8_t subindex, size_t num_bytes, std::error_code &ec) {
    std::array<uint8_t, 4> data{};
    const auto len = sdo_upload(index, subindex, std::span<uint8_t>(data.data(), num_bytes), ec);
    if (len == 0) {
      return 0;
    }
    // The typed accessors promise the exact width they were asked for; a
    // shorter response (e.g. read_u32 on a u16 object) would otherwise decode
    // to a silently-wrong value. Treat any size mismatch as a protocol error.
    if (len != num_bytes) {
      logger_.error("SDO upload 0x{:04X}:{:02X}: object is {} bytes, expected {}", index, subindex,
                    len, num_bytes);
      ec = std::make_error_code(std::errc::protocol_error);
      return 0;
    }
    return detail::canopen::get_le(data.data(), len);
  }

  uint8_t node_id_;
  send_fn send_;
  std::chrono::milliseconds sdo_timeout_;
  heartbeat_callback_fn on_heartbeat_;

  // SDO transaction state: sdo_mutex_ serializes whole transactions;
  // response_mutex_ / response_cv_ hand the parsed response from
  // process_frame() to the waiting transaction.
  std::mutex sdo_mutex_;
  mutable std::mutex response_mutex_;
  std::condition_variable response_cv_;
  bool awaiting_response_{false};
  uint16_t expected_index_{0};   // object address of the in-flight SDO request
  uint8_t expected_subindex_{0}; // (used to reject stale/unrelated responses)
  bool expected_segment_{false}; // in-flight request awaits an upload segment
  detail::canopen::SdoResponse response_{};
  uint32_t last_abort_code_{0};

  mutable std::mutex state_mutex_;
  std::unordered_map<uint8_t, NmtState> node_states_;

  std::mutex pdo_mutex_;
  std::unordered_map<uint32_t, pdo_callback_fn> pdo_callbacks_;
};

} // namespace espp
