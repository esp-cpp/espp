#pragma once

// espp::GamepadMapService -- view and edit an espp::GamepadProfiles store and
// watch a controller's input before / after mapping, as a transport-agnostic
// dispatcher module (detail/gamepad_map_protocol.hpp is the wire spec). Same
// contract as the other espp services: requests are handled under an internal
// mutex, the `send` callback runs with that mutex released, frames for other
// modules / reply-flagged frames are ignored so the service shares a stream.
//
// Wiring (one line per transport):
//
//   espp::GamepadMapService map_service({.send = [&](auto f) { usb.write_vendor(f); },
//                                        .profiles = &profiles});
//   dispatcher.register_module(map_service);   // module 11 + discovery metadata

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "dispatcher.hpp"
#include "stream_frame.hpp"

#include "base_component.hpp"
#include "detail/gamepad_map_protocol.hpp"
#include "gamepad_map.hpp"
#include "gamepad_profiles.hpp"
#include "gamepad_state.hpp"
#include "task.hpp"

namespace espp {

/**
 * @brief Serves a GamepadProfiles store over any framed byte stream so a host
 *        (the hosted gamepad map web console) can view and edit the maps and
 *        watch a controller's input live (dispatcher module 11 by default; see
 *        Config::module).
 *
 * @details The application tells the service which controller is connected
 *          (set_controller() / clear_controller()) and hands it every input it
 *          maps (publish_input(), raw and mapped). The service answers GET_INFO
 *          / GET_MAP / LIST_PROFILES from the store, applies SET_MAP /
 *          ERASE_PROFILE / RESET_ALL to it (every map is validated, every
 *          change saved), and while a host has the stream on, sends the latest
 *          input as INPUT events every period. The stream stops by itself when
 *          the host does not renew it within Config::stream_lease, so a closed
 *          browser tab does not leave it running.
 *
 *          When a change affects the map of the connected controller (its
 *          profile, or the default map it falls back to), the service calls
 *          Config::on_active_map_changed with the new map: the application
 *          swaps the map it applies. publish_input() is cheap when nobody is
 *          streaming (an atomic load), so it can be called for every report.
 *
 *          Several services (one per transport) may share one GamepadProfiles
 *          store; tell each about the controller and feed each the input.
 *
 * **Threading**: an internal mutex covers the parser and the controller state;
 * every outbound frame is serialized on a send mutex held across `send`, so
 * frames never interleave. `send` and `on_active_map_changed` must not
 * re-enter this object.
 *
 * \section gamepad_map_service_ex1 GamepadMapService Example
 * \snippet gamepad_map_webapp_example.cpp gamepad_map_webapp_example
 */
class GamepadMapService : public BaseComponent {
public:
  using Stream = espp::stream_frame::StreamParser;
  using Type = espp::detail::gamepad_map_protocol::Type;

  /// Default dispatcher module id (11). A routing key only: Config::module serves
  /// on any id, and hosts find it through discovery (by kProtocol).
  static constexpr uint8_t kModule = espp::detail::gamepad_map_protocol::kModule;
  /// Stable protocol identifier + version advertised through discovery.
  static constexpr const char *kProtocol = espp::detail::gamepad_map_protocol::kProtocol;
  static constexpr uint16_t kProtocolVersion = espp::detail::gamepad_map_protocol::kProtocolVersion;

  /// Transmits one encoded frame to the host.
  using send_fn = std::function<void(std::span<const uint8_t> frame)>;
  /// Called with the connected controller's new map after a host changed it.
  using map_changed_fn = std::function<void(const GamepadMap &map)>;

  /// Configuration for the GamepadMapService.
  struct Config {
    send_fn send{nullptr};              ///< Transmits an encoded frame (required).
    GamepadProfiles *profiles{nullptr}; ///< The store served (required; must outlive the service).
    /// Called (without the service's locks held) when a host's change gives the
    /// connected controller a new map.
    map_changed_fn on_active_map_changed{nullptr};
    /// Dispatcher module id this instance answers on (and stamps on every
    /// frame it sends). A routing key only (0x00..0xEF).
    uint8_t module{kModule};
    /// Shortest INPUT period a host may request (SET_STREAM is clamped to it).
    std::chrono::milliseconds min_stream_period{20};
    /// The stream stops when no SET_STREAM enable renews it for this long.
    std::chrono::milliseconds stream_lease{10000};
    /// The streaming task (started on the first SET_STREAM enable).
    Task::BaseConfig task_config{.name = "gamepad_map_stream", .stack_size_bytes = 4 * 1024};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /// @brief Construct the service.
  explicit GamepadMapService(const Config &config)
      : BaseComponent("GamepadMapService", config.log_level)
      , config_(config) {
    if (!config_.profiles)
      logger_.error("no GamepadProfiles store configured: every request will fail");
  }

  ~GamepadMapService() { stop_stream(); }

  /// @brief The dispatcher module id this service answers on (Config::module).
  uint8_t module_id() const { return config_.module; }

  /// @brief Discovery metadata for registering this service on a Dispatcher.
  Dispatcher::ModuleInfo module_info() const {
    return {.name = "Gamepad map",
            .app = "gamepad_map_console.html",
            .description = "Gamepad button / axis mapping and per-controller profiles",
            .protocol = kProtocol,
            .protocol_version = kProtocolVersion};
  }

  // --- application side ---

  /**
   * @brief A controller connected (or the connected one changed).
   * @return The map to apply to it (its profile's, else the default map).
   */
  GamepadMap set_controller(uint16_t vid, uint16_t pid, const std::string &name) {
    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const Controller c{vid, pid, name};
      changed = !controller_ || !(*controller_ == c);
      controller_ = c;
    }
    if (changed)
      push_info();
    return config_.profiles ? config_.profiles->map_for(vid, pid, name) : GamepadMap{};
  }

  /// @brief The controller disconnected (INPUT then reports no controller).
  void clear_controller() {
    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      changed = controller_.has_value();
      controller_.reset();
    }
    if (changed)
      push_info();
  }

  /// @brief The map the connected controller should use now (default map when none is connected).
  GamepadMap active_map() const {
    if (!config_.profiles)
      return {};
    const auto c = controller();
    return c ? config_.profiles->map_for(c->vid, c->pid, c->name) : config_.profiles->default_map();
  }

  /**
   * @brief Report one input: what the controller sent and what it was mapped
   *        to. Cheap (an atomic load) while no host is streaming.
   */
  void publish_input(const GamepadState &raw, const GamepadState &mapped) {
    if (!streaming_.load(std::memory_order_relaxed))
      return;
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (raw != latest_raw_ || mapped != latest_mapped_)
      input_changed_ = true;
    latest_raw_ = raw;
    latest_mapped_ = mapped;
  }

  /// @brief Whether INPUT events are being sent.
  bool streaming() const { return streaming_.load(); }

  /// @brief Stop streaming (also done by the destructor, SET_STREAM 0 and the lease).
  void stop_stream() {
    std::unique_ptr<Task> task;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      streaming_.store(false);
      task = std::move(task_);
    }
    if (task)
      task->stop();
  }

  // --- transport side ---

  /**
   * @brief Dispatcher entry point: handle one routed frame. Frames for other
   *        modules and reply-flagged frames are ignored, so this can be
   *        registered directly: `dispatcher.register_module(service)`.
   */
  void handle(const espp::stream_frame::Frame &frame) {
    if (frame.module != module_id() || frame.is_reply())
      return;
    handle_frame(frame.type, frame.payload, frame.correlation);
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
   * @param correlation The request frame's correlation id, if it carried one;
   *        every reply echoes it (streamed events carry none).
   * @return true if the type belongs to the gamepad map protocol (a reply was
   *         sent), false if it was ignored.
   */
  bool handle_frame(uint8_t type, std::span<const uint8_t> payload,
                    std::optional<uint16_t> correlation = std::nullopt) {
    namespace proto = espp::detail::gamepad_map_protocol;
    const auto request = static_cast<Type>(type);
    switch (request) {
    case Type::GetInfo:
    case Type::GetMap:
    case Type::SetMap:
    case Type::EraseProfile:
    case Type::ListProfiles:
    case Type::SetStream:
    case Type::ResetAll:
      break;
    default:
      return false; // not a gamepad map request: ignore so the service can share a stream
    }
    if (!config_.profiles) {
      send_error(type, std::errc::not_supported, "no profile store configured", correlation);
      return true;
    }
    auto &profiles = *config_.profiles;
    switch (request) {
    case Type::GetInfo:
      send_frame(proto::build_frame(Type::Info, proto::encode_info(build_info()), module_id(),
                                    correlation));
      break;
    case Type::GetMap: {
      const auto target = proto::decode_get_map(payload);
      if (!target) {
        send_error(type, std::errc::invalid_argument, "malformed GET_MAP", correlation);
        break;
      }
      proto::Target resolved = *target;
      GamepadMap map;
      bool exists = true;
      if (target->profile) {
        if (const auto p = profiles.profile(target->vid, target->pid, target->name)) {
          map = p->map;
          resolved.name = p->name; // as stored
        } else {
          exists = false; // no such profile: the controller uses the default map today
          map = profiles.default_map();
        }
      } else {
        map = profiles.default_map();
      }
      send_frame(proto::build_frame(Type::Map, proto::encode_map_reply(resolved, exists, map),
                                    module_id(), correlation));
      break;
    }
    case Type::SetMap: {
      const auto req = proto::decode_set_map(payload);
      if (!req) {
        send_error(type, std::errc::invalid_argument, "malformed SET_MAP", correlation);
        break;
      }
      const auto &[target, map] = *req;
      std::string why;
      if (!map.valid(&why)) {
        send_error(type, std::errc::invalid_argument, "invalid map: " + why, correlation);
        break;
      }
      const GamepadMap before = active_map();
      std::error_code ec;
      const bool saved =
          target.profile
              ? profiles.set_profile(
                    {.vid = target.vid, .pid = target.pid, .name = target.name, .map = map}, ec)
              : profiles.set_default_map(map, ec);
      if (!saved) {
        send_error(type, std::errc::io_error,
                   "could not save the map: " + (ec ? ec.message() : std::string("unknown error")),
                   correlation, ec);
        break;
      }
      ok(type, correlation);
      notify_if_changed(before);
      break;
    }
    case Type::EraseProfile: {
      const auto target = proto::decode_erase_profile(payload);
      if (!target) {
        send_error(type, std::errc::invalid_argument, "malformed ERASE_PROFILE", correlation);
        break;
      }
      const GamepadMap before = active_map();
      std::error_code ec;
      if (!profiles.erase_profile(target->vid, target->pid, target->name, ec)) {
        if (ec)
          send_error(type, std::errc::io_error, "could not save: " + ec.message(), correlation, ec);
        else
          send_error(type, std::errc::no_such_file_or_directory, "no such profile", correlation);
        break;
      }
      ok(type, correlation);
      notify_if_changed(before);
      break;
    }
    case Type::ListProfiles: {
      const auto stored = profiles.profiles();
      std::vector<proto::ProfileSummary> list;
      list.reserve(stored.size());
      std::transform(
          stored.begin(), stored.end(), std::back_inserter(list), [](const GamepadProfile &p) {
            return proto::ProfileSummary{
                .vid = p.vid,
                .pid = p.pid,
                .name = p.name,
                .rule_count = static_cast<uint8_t>(std::min<size_t>(p.map.rules.size(), 255))};
          });
      size_t encoded = 0;
      auto reply = proto::encode_profiles(list, max_payload(), &encoded);
      if (encoded < list.size())
        logger_.warn("PROFILES payload full: listing {} of {} profiles", encoded, list.size());
      send_frame(proto::build_frame(Type::Profiles, reply, module_id(), correlation));
      break;
    }
    case Type::SetStream: {
      const auto req = proto::decode_set_stream(payload);
      if (!req) {
        send_error(type, std::errc::invalid_argument,
                   "malformed SET_STREAM (expected u8 enable, u16 period_ms)", correlation);
        break;
      }
      if (req->enable)
        start_stream(std::chrono::milliseconds(req->period_ms));
      else
        stop_stream();
      ok(type, correlation);
      break;
    }
    case Type::ResetAll: {
      const GamepadMap before = active_map();
      std::error_code ec;
      if (!profiles.reset(ec)) {
        send_error(type, std::errc::io_error, "could not save: " + ec.message(), correlation, ec);
        break;
      }
      ok(type, correlation);
      notify_if_changed(before);
      break;
    }
    default:
      break;
    }
    return true;
  }

protected:
  struct Controller {
    uint16_t vid{0};
    uint16_t pid{0};
    std::string name{};
    bool operator==(const Controller &) const = default;
  };

  /// Tell a streaming host about a controller change right away (unsolicited
  /// INFO, no correlation id); a host that is not streaming asks with GET_INFO.
  void push_info() {
    namespace proto = espp::detail::gamepad_map_protocol;
    if (!config_.profiles || !streaming_.load())
      return;
    send_frame(proto::build_frame(Type::Info, proto::encode_info(build_info()), module_id()));
  }

  std::optional<Controller> controller() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return controller_;
  }

  espp::detail::gamepad_map_protocol::Info build_info() const {
    namespace proto = espp::detail::gamepad_map_protocol;
    proto::Info info{
        .button_count = static_cast<uint8_t>(GamepadState::BUTTON_COUNT),
        .axis_count = static_cast<uint8_t>(GamepadState::AXIS_COUNT),
        .max_rules = static_cast<uint8_t>(std::min<size_t>(GamepadMap::MAX_RULES, 255)),
        .max_profiles =
            static_cast<uint8_t>(std::min<size_t>(config_.profiles->max_profiles(), 255)),
        .profile_count =
            static_cast<uint8_t>(std::min<size_t>(config_.profiles->profiles().size(), 255))};
    if (const auto c = controller()) {
      info.controller = proto::Controller{
          .vid = c->vid,
          .pid = c->pid,
          .name = c->name,
          .has_profile = config_.profiles->profile(c->vid, c->pid, c->name).has_value()};
    }
    return info;
  }

  void notify_if_changed(const GamepadMap &before) {
    if (!config_.on_active_map_changed || !controller())
      return;
    const GamepadMap now = active_map();
    if (now != before)
      config_.on_active_map_changed(now);
  }

  void ok(uint8_t request_type, std::optional<uint16_t> correlation) {
    namespace proto = espp::detail::gamepad_map_protocol;
    send_frame(
        proto::build_frame(Type::Ok, proto::encode_ok(request_type), module_id(), correlation));
  }

  /// The payload cap: the stream_frame payload limit.
  static constexpr size_t max_payload() { return espp::stream_frame::kMaxPayloadSize; }

  void start_stream(std::chrono::milliseconds period) {
    std::lock_guard<std::mutex> lock(mutex_);
    period_ms_.store(std::max(period, config_.min_stream_period).count());
    lease_until_.store(now_ms() + config_.stream_lease.count());
    {
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      input_changed_ = true; // the first event after (re)enabling is always "changed"
    }
    streaming_.store(true);
    if (task_)
      return; // already running: the new period and lease apply on its next wake
    task_ = std::make_unique<Task>(
        Task::Config{.callback = [this](std::mutex &m, std::condition_variable &cv,
                                        bool &notified) { return stream_step(m, cv, notified); },
                     .task_config = config_.task_config});
    task_->start();
  }

  /// One streaming period: send the latest input (no correlation id: events,
  /// not replies), end the stream when its lease ran out, then wait one period.
  bool stream_step(std::mutex &m, std::condition_variable &cv, bool &notified) {
    namespace proto = espp::detail::gamepad_map_protocol;
    if (streaming_.load()) {
      if (now_ms() > lease_until_.load()) {
        logger_.info("INPUT stream lease expired; stopping the stream");
        streaming_.store(false);
      } else {
        proto::Input in;
        in.connected = controller().has_value();
        {
          std::lock_guard<std::mutex> lock(input_mutex_);
          in.raw = latest_raw_;
          in.mapped = latest_mapped_;
          in.changed = input_changed_;
          input_changed_ = false;
        }
        send_frame(proto::build_frame(Type::Input, proto::encode_input(in), module_id()));
      }
    }
    // A stopped stream (lease ran out) leaves the task idle at a slow tick until
    // the next SET_STREAM, which takes effect on its next wake.
    const auto wait = streaming_.load() ? std::chrono::milliseconds(period_ms_.load())
                                        : std::chrono::milliseconds(250);
    std::unique_lock<std::mutex> lock(m);
    cv.wait_for(lock, wait, [&notified] { return notified; });
    notified = false; // consumed, under the mutex, per the Task contract
    return false;
  }

  static int64_t now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
  }

  /// Transmit a frame. Serialized on send_mutex_ (held across the callback) so
  /// an event and a reply never interleave. Never called with mutex_ held.
  void send_frame(const std::vector<uint8_t> &frame) {
    if (frame.empty())
      return;
    std::lock_guard<std::mutex> send_lock(send_mutex_);
    if (!config_.send) {
      logger_.warn_rate_limited("no send function configured; dropping a {}-byte frame",
                                frame.size());
      return;
    }
    config_.send(frame);
  }

  void send_error(uint8_t request_type, std::errc errc, std::string_view message,
                  std::optional<uint16_t> correlation = std::nullopt, std::error_code cause = {}) {
    namespace proto = espp::detail::gamepad_map_protocol;
    logger_.warn("{} (type 0x{:02x})", message, request_type);
    // a generic-category cause is already a POSIX errno; anything else reports
    // the std::errc the service chose
    const uint32_t code = static_cast<uint32_t>(cause && cause.category() == std::generic_category()
                                                    ? cause.value()
                                                    : std::make_error_code(errc).value());
    send_frame(proto::build_frame(Type::Error, proto::encode_error(request_type, code, message),
                                  module_id(), correlation));
  }

private:
  Config config_;
  mutable std::mutex mutex_;      ///< guards the parser, controller_ and task_
  mutable std::mutex send_mutex_; ///< serializes every outbound frame across `send`
  std::mutex input_mutex_;        ///< guards the latest input
  Stream parser_;
  std::optional<Controller> controller_{};
  std::unique_ptr<Task> task_;
  std::atomic<bool> streaming_{false};
  std::atomic<int64_t> period_ms_{20};
  std::atomic<int64_t> lease_until_{0};
  GamepadState latest_raw_{};
  GamepadState latest_mapped_{};
  bool input_changed_{true};
};

// Compile-time check that the service keeps satisfying the dispatcher's module
// contract (module_id() / module_info() / handle(frame)).
static_assert(DispatcherModuleConcept<GamepadMapService>);

} // namespace espp
