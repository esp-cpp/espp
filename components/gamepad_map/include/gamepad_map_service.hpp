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
#include <utility>
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
 *          swaps the map it applies. Likewise, when a change gives the
 *          connected controller different rumble options (see RumbleOptions),
 *          the service calls Config::on_active_rumble_changed with them: the
 *          application applies the new options to the rumble it forwards.
 *          publish_input() only copies the state under a lock (no I/O), so it
 *          can be called for every report.
 *
 *          Several services (one per transport) may share one GamepadProfiles
 *          store; tell each about the controller and feed each the input.
 *          Their changes and notifications are serialized on the store's
 *          change_mutex(), so the last notification always carries the map
 *          (and rumble options) the store holds.
 *
 * **Threading**: an internal mutex covers the parser, the controller state and
 * the stream task; every outbound frame is serialized on a send mutex held
 * across `send`, so frames never interleave. set_controller(),
 * clear_controller() and publish_input() never send (an INFO for a controller
 * change goes out from the stream task), so they are safe to call from a
 * radio / host task. `send`, `on_active_map_changed` and
 * `on_active_rumble_changed` must not re-enter this object (nor change the
 * store).
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
  /// Called with the connected controller's new rumble options after a host changed them.
  using rumble_changed_fn = std::function<void(const RumbleOptions &rumble)>;

  /// Configuration for the GamepadMapService.
  struct Config {
    send_fn send{nullptr};              ///< Transmits an encoded frame (required).
    GamepadProfiles *profiles{nullptr}; ///< The store served (required; must outlive the service).
    /// Called (without the service's locks held) when a host's change gives the
    /// connected controller a new map.
    map_changed_fn on_active_map_changed{nullptr};
    /// Called (without the service's locks held, under the store's
    /// change_mutex()) when a host's change gives the connected controller
    /// different rumble options.
    rumble_changed_fn on_active_rumble_changed{nullptr};
    /// Dispatcher module id this instance answers on (and stamps on every
    /// frame it sends). A routing key only (0x00..0xEF).
    uint8_t module{kModule};
    /// Shortest INPUT period a host may request (SET_STREAM is clamped to it).
    std::chrono::milliseconds min_stream_period{20};
    /// Longest INPUT period a host may request (SET_STREAM is clamped to it).
    std::chrono::milliseconds max_stream_period{1000};
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
            .description =
                "Gamepad button / axis mapping, rumble options and per-controller profiles",
            .protocol = kProtocol,
            .protocol_version = kProtocolVersion};
  }

  // --- application side ---

  /**
   * @brief A controller connected (or the connected one changed).
   * @details Call active_rumble() right after for its rumble options, under the
   *          same lock your change callbacks take, so a host's change cannot
   *          slip in between. With several services on one store, call
   *          set_controller() on every one of them first, then look the map
   *          and rumble options up once (from any of them), all under that
   *          lock: a change through any service then either lands before the
   *          lookup or notifies.
   * @return The map to apply to it (its profile's, else the default map).
   */
  GamepadMap set_controller(uint16_t vid, uint16_t pid, const std::string &name) {
    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const Controller c{vid, pid, name};
      changed = !controller_ || !(*controller_ == c);
      controller_ = c;
      if (changed)
        ++controller_generation_; // a change() in flight notifies for this controller
    }
    if (changed) {
      reset_input(); // nothing from the previous controller carries over
      info_pending_ = true;
    }
    return config_.profiles ? config_.profiles->map_for(vid, pid, name) : GamepadMap{};
  }

  /// @brief The controller disconnected (INPUT then reports no controller).
  void clear_controller() {
    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      changed = controller_.has_value();
      controller_.reset();
      if (changed)
        ++controller_generation_;
    }
    if (changed) {
      reset_input();
      info_pending_ = true;
    }
  }

  /// @brief The map the connected controller should use now (default map when none is connected).
  GamepadMap active_map() const {
    if (!config_.profiles)
      return {};
    const auto c = controller();
    return c ? config_.profiles->map_for(c->vid, c->pid, c->name) : config_.profiles->default_map();
  }

  /// @brief The rumble options the connected controller should use now (the
  ///        default ones when none is connected). Apply them once, where the
  ///        host's raw rumble becomes the controller's motor command.
  RumbleOptions active_rumble() const {
    if (!config_.profiles)
      return {};
    const auto c = controller();
    return c ? config_.profiles->rumble_for(c->vid, c->pid, c->name)
             : config_.profiles->default_rumble();
  }

  /**
   * @brief Report one input: what the controller sent and what it was mapped
   *        to. Copies 32 bytes under a lock (always, so a stream that starts
   *        later begins from the real state); never sends.
   */
  void publish_input(const GamepadState &raw, const GamepadState &mapped) {
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
      task_active_ = false;
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
    // requests without a payload take none: a longer one is a newer protocol's
    // request this firmware does not understand (never act on it)
    if ((request == Type::GetInfo || request == Type::ListProfiles || request == Type::ResetAll) &&
        !payload.empty()) {
      send_error(type, std::errc::invalid_argument, "unexpected payload", correlation);
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
      RumbleOptions rumble;
      bool exists = true;
      if (target->profile) {
        if (const auto p = profiles.profile(target->vid, target->pid, target->name)) {
          map = p->map;
          rumble = p->rumble;
          resolved.name = p->name; // as stored
        } else {
          // no such profile: the controller uses the default map and rumble today
          exists = false;
          map = profiles.default_map();
          rumble = profiles.default_rumble();
        }
      } else {
        map = profiles.default_map();
        rumble = profiles.default_rumble();
      }
      send_frame(proto::build_frame(Type::Map,
                                    proto::encode_map_reply(resolved, exists, map, rumble),
                                    module_id(), correlation));
      break;
    }
    case Type::SetMap: {
      const auto req = proto::decode_set_map(payload);
      if (!req) {
        send_error(type, std::errc::invalid_argument, "malformed SET_MAP", correlation);
        break;
      }
      const auto &target = req->target;
      const auto &map = req->map;
      const auto &rumble = req->rumble;
      std::string why;
      if (!map.valid(&why)) {
        send_error(type, std::errc::invalid_argument, "invalid map: " + why, correlation);
        break;
      }
      if (rumble && !rumble->valid(&why)) {
        send_error(type, std::errc::invalid_argument, "invalid rumble options: " + why,
                   correlation);
        break;
      }
      std::error_code ec;
      // with a rumble block, the map and the options are saved together; without
      // one (an older host) the stored options are kept
      const bool saved = change([&] {
        if (target.profile) {
          return rumble ? profiles.set_profile({.vid = target.vid,
                                                .pid = target.pid,
                                                .name = target.name,
                                                .map = map,
                                                .rumble = *rumble},
                                               ec)
                        : profiles.set_profile_map(target.vid, target.pid, target.name, map, ec);
        }
        return rumble ? profiles.set_defaults(map, *rumble, ec) : profiles.set_default_map(map, ec);
      });
      if (!saved) {
        send_error(type, std::errc::io_error,
                   "could not save the map: " + (ec ? ec.message() : std::string("unknown error")),
                   correlation, ec);
        break;
      }
      ok(type, correlation);
      break;
    }
    case Type::EraseProfile: {
      const auto target = proto::decode_erase_profile(payload);
      if (!target) {
        send_error(type, std::errc::invalid_argument, "malformed ERASE_PROFILE", correlation);
        break;
      }
      std::error_code ec;
      if (!change(
              [&] { return profiles.erase_profile(target->vid, target->pid, target->name, ec); })) {
        if (ec)
          send_error(type, std::errc::io_error, "could not save: " + ec.message(), correlation, ec);
        else
          send_error(type, std::errc::no_such_file_or_directory, "no such profile", correlation);
        break;
      }
      ok(type, correlation);
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
      std::error_code ec;
      if (!change([&] { return profiles.reset(ec); })) {
        send_error(type, std::errc::io_error, "could not save: " + ec.message(), correlation, ec);
        break;
      }
      ok(type, correlation);
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

  /// Forget the latest input (a controller changed): the next INPUT shows a
  /// neutral state until the new controller reports.
  void reset_input() {
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_raw_ = {};
    latest_mapped_ = {};
    input_changed_ = true;
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
            static_cast<uint8_t>(std::min<size_t>(config_.profiles->profile_count(), 255)),
        .rumble_options = true};
    if (const auto c = controller()) {
      info.controller =
          proto::Controller{.vid = c->vid,
                            .pid = c->pid,
                            .name = c->name,
                            .has_profile = config_.profiles->has_profile(c->vid, c->pid, c->name)};
    }
    return info;
  }

  /// The connected controller and its generation, read together.
  std::pair<std::optional<Controller>, uint32_t> controller_snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {controller_, controller_generation_.load()};
  }

  /// The map / rumble options for a controller (none: the default ones).
  GamepadMap map_of(const std::optional<Controller> &c) const {
    return c ? config_.profiles->map_for(c->vid, c->pid, c->name) : config_.profiles->default_map();
  }
  RumbleOptions rumble_of(const std::optional<Controller> &c) const {
    return c ? config_.profiles->rumble_for(c->vid, c->pid, c->name)
             : config_.profiles->default_rumble();
  }

  /// Apply one change to the store (`save` returns whether it was saved) and,
  /// when it gives the connected controller a different map or different
  /// rumble options, tell the application -- all under the store's change
  /// mutex, so services sharing a store change and notify one at a time and
  /// the last notification carries what the store holds.
  ///
  /// Whether to notify is decided after the save, for the controller
  /// connected then: one that connected while the save ran (its values looked
  /// up before the save landed) is told too. The controller is re-checked
  /// after the callbacks: if another one connected while they ran (it may
  /// have been handed its values just before a callback delivered the
  /// previous controller's), the callbacks run again for it, so the last
  /// notification always matches the connected controller.
  template <typename Save> bool change(Save save) {
    std::lock_guard<std::mutex> lock(config_.profiles->change_mutex());
    const auto [before, before_generation] = controller_snapshot();
    const GamepadMap map_before = config_.on_active_map_changed ? map_of(before) : GamepadMap{};
    const RumbleOptions rumble_before =
        config_.on_active_rumble_changed ? rumble_of(before) : RumbleOptions{};
    if (!save())
      return false;
    uint32_t seen = before_generation;
    // bounded: a controller would have to reconnect during every round
    for (int round = 0; round < 4; ++round) {
      const auto [now, generation] = controller_snapshot();
      if (!now)
        break; // nothing connected: the next connect looks its values up
      const bool reconnected = generation != seen;
      if (config_.on_active_map_changed) {
        const GamepadMap map = map_of(now);
        if (reconnected || map != map_before)
          config_.on_active_map_changed(map);
      }
      if (config_.on_active_rumble_changed) {
        const RumbleOptions rumble = rumble_of(now);
        if (reconnected || rumble != rumble_before)
          config_.on_active_rumble_changed(rumble);
      }
      if (controller_snapshot().second == generation)
        break;
      seen = generation; // another controller connected meanwhile: tell it too
    }
    return true;
  }

  void ok(uint8_t request_type, std::optional<uint16_t> correlation) {
    namespace proto = espp::detail::gamepad_map_protocol;
    send_frame(
        proto::build_frame(Type::Ok, proto::encode_ok(request_type), module_id(), correlation));
  }

  /// The payload cap: the stream_frame payload limit.
  static constexpr size_t max_payload() { return espp::stream_frame::kMaxPayloadSize; }

  void start_stream(std::chrono::milliseconds period) {
    std::unique_ptr<Task> finished;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      period_ms_.store(
          std::clamp(period, config_.min_stream_period, config_.max_stream_period).count());
      lease_until_.store(now_ms() + config_.stream_lease.count());
      if (!streaming_.load()) {
        // the first event after (re)enabling is a full picture
        std::lock_guard<std::mutex> input_lock(input_mutex_);
        input_changed_ = true;
        info_pending_ = true;
      }
      streaming_.store(true);
      if (task_ && task_active_)
        return;                    // running: the new period and lease apply on its next wake
      finished = std::move(task_); // ended itself (lease ran out): join it below
      task_active_ = true;
      task_ = std::make_unique<Task>(
          Task::Config{.callback = [this](std::mutex &m, std::condition_variable &cv,
                                          bool &notified) { return stream_step(m, cv, notified); },
                       .task_config = config_.task_config});
      task_->start();
    }
    if (finished)
      finished->stop(); // it already returned: this only joins it
  }

  /// One streaming period: send a pending INFO and the latest input (no
  /// correlation id: events, not replies), then wait one period. When the
  /// lease has run out (or the stream was stopped) the task ends itself; the
  /// next SET_STREAM starts a new one.
  bool stream_step(std::mutex &m, std::condition_variable &cv, bool &notified) {
    namespace proto = espp::detail::gamepad_map_protocol;
    if (!streaming_.load() || now_ms() > lease_until_.load()) {
      // decided under mutex_, where start_stream() renews the lease, so a
      // renewal that just landed is never undone
      std::lock_guard<std::mutex> lock(mutex_);
      if (!streaming_.load() || now_ms() > lease_until_.load()) {
        if (streaming_.load())
          logger_.info("INPUT stream lease expired; stopping the stream");
        streaming_.store(false);
        task_active_ = false;
        return true; // end this task
      }
    }
    if (info_pending_.exchange(false))
      send_frame(proto::build_frame(Type::Info, proto::encode_info(build_info()), module_id()));
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
    std::unique_lock<std::mutex> lock(m);
    cv.wait_for(lock, std::chrono::milliseconds(period_ms_.load()),
                [&notified] { return notified; });
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
  /// Bumped whenever the connected controller changes (under mutex_), so
  /// change() can tell that one connected while it saved.
  std::atomic<uint32_t> controller_generation_{0};
  std::unique_ptr<Task> task_;
  std::atomic<bool> streaming_{false};
  bool task_active_{false};               ///< task_ is running (guarded by mutex_)
  std::atomic<bool> info_pending_{false}; ///< send INFO on the next stream tick
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
