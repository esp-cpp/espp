#pragma once

// espp::SystemService -- device identity / status and reboot control as a
// transport-agnostic dispatcher module (detail/system_protocol.hpp is the wire
// spec). It follows the same contract as espp::OtaService / CoreDumpService:
// requests are handled and the reply built under an internal mutex, the
// `send` callback always runs after that mutex is released, and frames for
// other modules / reply-flagged frames are ignored so the service coexists
// with other protocols on one stream.
//
// Wiring (one line per transport):
//
//   espp::SystemService system_service({.send = [&](auto f) { usb.write_vendor(f); }});
//   dispatcher.register_module(system_service);   // module 7 + discovery metadata

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "dispatcher.hpp"
#include "stream_frame.hpp"

#include "base_component.hpp"
#include "detail/system_protocol.hpp"
#include "system_control.hpp"
#include "system_info.hpp"

namespace espp {

/**
 * @brief Serves espp::SystemInfo and espp::SystemControl over any framed byte
 *        stream (dispatcher module 7 by default; see Config::module).
 *
 * GET_INFO answers with a tagged-record snapshot (see
 * detail/system_protocol.hpp; hosts skip tags they do not know). REBOOT and
 * REBOOT_TO_BOOTLOADER reply OK first and restart after the requested delay
 * (at least Config::min_restart_delay, so the reply leaves the transport)
 * from a detached thread -- the same pattern as OtaService's post-update
 * restart. Both are guarded: Config::allow_reboot / allow_bootloader switch
 * them off (ERROR "not permitted"), the optional Config::on_reboot_request
 * callback can veto a specific request (an application with a motor running
 * can refuse or defer), and REBOOT_TO_BOOTLOADER is refused with "not
 * supported" on chips without a software download-mode path (classic ESP32).
 * The INFO capabilities record tells a host up front which of the two it may
 * offer.
 *
 * **Threading**: an internal mutex covers the parser and request handling;
 * the `send` callback and the veto callback run after it is released. Drive
 * one instance from one context per byte stream (a Dispatcher /
 * DispatcherWorker, or a single task calling feed()).
 *
 * \section system_service_ex1 SystemService Example
 * \snippet system_example.cpp system_example
 */
class SystemService : public BaseComponent {
public:
  using Stream = espp::stream_frame::StreamParser;
  using Type = espp::detail::system_protocol::Type;

  /// Default dispatcher module id (7). Only a routing key: Config::module serves
  /// on any id, and the hosted system console finds it through discovery (by
  /// kProtocol).
  static constexpr uint8_t kModule = espp::detail::system_protocol::kModule;
  /// Stable protocol identifier + version advertised through discovery.
  static constexpr const char *kProtocol = espp::detail::system_protocol::kProtocol;
  static constexpr uint16_t kProtocolVersion = espp::detail::system_protocol::kProtocolVersion;

  /// Which restart a host asked for (passed to Config::on_reboot_request).
  enum class RebootKind : uint8_t {
    Reboot,     ///< plain restart
    Bootloader, ///< restart into the ROM download mode
  };

  /// Transmits one encoded reply frame to the host.
  using send_fn = std::function<void(std::span<const uint8_t> frame)>;
  /// Asked (outside the lock) before a permitted reboot is acknowledged;
  /// return false to veto it (the host gets ERROR "refused by the application").
  using reboot_request_fn = std::function<bool(RebootKind kind)>;

  /// Configuration for the SystemService.
  struct Config {
    send_fn send{nullptr}; ///< Transmits an encoded reply frame (required).
    /// Dispatcher module id this instance answers on (and stamps on its
    /// replies). A routing key only: hosts find whichever id is chosen through
    /// discovery (by kProtocol), so any id 0x00..0xEF is fine.
    uint8_t module{kModule};
    bool allow_reboot{true};     ///< Serve REBOOT (else ERROR "not permitted").
    bool allow_bootloader{true}; ///< Serve REBOOT_TO_BOOTLOADER (else ERROR "not permitted").
    /// Optional veto for a specific reboot request; called after the allow_*
    /// checks, outside the lock. nullptr = every permitted request proceeds.
    reboot_request_fn on_reboot_request{nullptr};
    /// Lower bound on the delay between the OK reply and the restart, so the
    /// reply reaches the host even when it asked for 0 ms.
    std::chrono::milliseconds min_restart_delay{250};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /// @brief Construct the service.
  explicit SystemService(const Config &config)
      : BaseComponent("SystemService", config.log_level)
      , config_(config) {}

  /// @brief The dispatcher module id this service answers on (Config::module).
  uint8_t module_id() const { return config_.module; }

  /// @brief Discovery metadata for registering this service on a Dispatcher.
  Dispatcher::ModuleInfo module_info() const {
    return {.name = "System",
            .app = "system_console.html",
            .description = "Device info, reboot and bootloader entry",
            .protocol = kProtocol,
            .protocol_version = kProtocolVersion};
  }

  /// @brief The capabilities bitmask GET_INFO reports (kCapReboot / kCapBootloader).
  uint32_t capabilities() const {
    namespace proto = espp::detail::system_protocol;
    uint32_t caps = 0;
    if (config_.allow_reboot)
      caps |= proto::kCapReboot;
    if (config_.allow_bootloader && SystemControl::bootloader_reboot_supported())
      caps |= proto::kCapBootloader;
    return caps;
  }

  /// @brief Build the INFO payload (a SystemInfo snapshot as tagged records).
  std::vector<uint8_t> build_info() const {
    namespace proto = espp::detail::system_protocol;
    const SystemInfo::Snapshot s = SystemInfo::collect();
    proto::InfoBuilder b;
    b.str(proto::InfoTag::ChipModel, s.chip_model)
        .u16(proto::InfoTag::ChipRevision, s.chip_revision)
        .u8(proto::InfoTag::Cores, s.cores)
        .u32(proto::InfoTag::ChipFeatures, s.chip_features)
        .str(proto::InfoTag::IdfVersion, s.idf_version)
        .str(proto::InfoTag::ProjectName, s.project_name)
        .str(proto::InfoTag::AppVersion, s.app_version)
        .str(proto::InfoTag::BuildDate, s.build_date)
        .str(proto::InfoTag::BuildTime, s.build_time)
        .bytes(proto::InfoTag::ElfSha256, s.elf_sha256)
        .str(proto::InfoTag::RunningPartition, s.running_partition)
        .str(proto::InfoTag::BootPartition, s.boot_partition)
        .u8(proto::InfoTag::OtaState, s.ota_state)
        .u8(proto::InfoTag::ResetReason, s.reset_reason)
        .u64(proto::InfoTag::UptimeMs, s.uptime_ms)
        .bytes(proto::InfoTag::Mac, s.mac)
        .u32(proto::InfoTag::FlashSize, s.flash_size)
        .u32(proto::InfoTag::PsramSize, s.psram_size)
        .u32(proto::InfoTag::CpuMhz, s.cpu_mhz)
        .u32(proto::InfoTag::FreeHeap, s.free_heap)
        .u32(proto::InfoTag::MinFreeHeap, s.min_free_heap)
        .u32(proto::InfoTag::Capabilities, capabilities());
    return b.take();
  }

  /**
   * @brief Dispatcher entry point: handle one routed frame. Frames for other
   *        modules and reply-flagged frames are ignored, so this can be
   *        registered directly: `dispatcher.register_module(service)`.
   */
  void handle(const espp::stream_frame::Frame &frame) {
    if (frame.module != module_id() || frame.is_reply())
      return;
    handle_frame(frame.type, frame.payload);
  }

  /// @brief Feed received transport bytes (standalone use, without a Dispatcher).
  void feed(std::span<const uint8_t> data) {
    std::vector<espp::stream_frame::Frame> frames;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      frames = parser_.feed(data);
    }
    for (const auto &frame : frames)
      handle(frame);
  }

  /// @brief Discard any partially-buffered frame bytes (standalone feed() use).
  void reset_parser() {
    std::lock_guard<std::mutex> lock(mutex_);
    parser_.reset();
  }

  /**
   * @brief Handle one already-parsed request frame.
   * @return true if the type belongs to the system protocol (a reply was
   *         sent), false if it was ignored.
   * @note The `send` and veto callbacks run after the internal mutex is released.
   */
  bool handle_frame(uint8_t type, std::span<const uint8_t> payload) {
    namespace proto = espp::detail::system_protocol;
    std::error_code ec;
    switch (static_cast<Type>(type)) {
    case Type::GetInfo: {
      std::vector<uint8_t> info;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        info = build_info();
      }
      send(proto::build_frame(Type::Info, info, module_id()));
      return true;
    }
    case Type::Reboot:
    case Type::RebootToBootloader: {
      const bool bootloader = static_cast<Type>(type) == Type::RebootToBootloader;
      const auto delay = proto::decode_delay(payload);
      if (!delay) {
        send_error(type, std::errc::invalid_argument, "malformed request (expected u16 delay_ms)");
        return true;
      }
      const bool allowed = bootloader ? config_.allow_bootloader : config_.allow_reboot;
      if (!allowed) {
        send_error(type, std::errc::operation_not_permitted,
                   bootloader ? "reboot into bootloader is disabled on this device"
                              : "reboot is disabled on this device");
        return true;
      }
      if (bootloader && !SystemControl::bootloader_reboot_supported()) {
        send_error(type, std::errc::operation_not_supported,
                   "this chip has no software path into download mode (use the BOOT strap)");
        return true;
      }
      // the veto runs outside the lock: the application may take its own locks
      if (config_.on_reboot_request &&
          !config_.on_reboot_request(bootloader ? RebootKind::Bootloader : RebootKind::Reboot)) {
        send_error(type, std::errc::operation_canceled,
                   "refused by the application (try again later)");
        return true;
      }
      const auto wait = std::max(std::chrono::milliseconds(*delay), config_.min_restart_delay);
      // reply first, then restart from a detached thread so the reply leaves
      // the transport and the caller's task (the transport worker) is never blocked
      send(proto::build_frame(Type::Ok, proto::encode_ok(type), module_id()));
      logger_.info("{} in {} ms", bootloader ? "rebooting into the bootloader" : "rebooting",
                   wait.count());
      if (bootloader)
        SystemControl::reboot_to_bootloader_after(wait, ec);
      else
        SystemControl::reboot_after(wait);
      return true;
    }
    default:
      // not a system request: ignore so the service can share a stream
      return false;
    }
  }

protected:
  /// Transmit an encoded frame. Must be called WITHOUT the mutex held.
  void send(const std::vector<uint8_t> &frame) {
    if (frame.empty())
      return;
    if (!config_.send) {
      logger_.warn("no send function configured; dropping a {}-byte reply", frame.size());
      return;
    }
    config_.send(frame);
  }

  void send_error(uint8_t request_type, std::errc errc, std::string_view message) {
    namespace proto = espp::detail::system_protocol;
    logger_.warn("{} (type 0x{:02x})", message, request_type);
    send(proto::build_frame(Type::Error,
                            proto::encode_error(request_type, static_cast<uint32_t>(errc), message),
                            module_id()));
  }

private:
  Config config_;
  mutable std::mutex mutex_;
  Stream parser_;
};

// Compile-time check that the service keeps satisfying the dispatcher's module
// contract (module_id() / module_info() / handle(frame)).
static_assert(DispatcherModuleConcept<SystemService>);

} // namespace espp
