#pragma once

// espp::MonitorService -- heap-region and task statistics (espp::HeapMonitor /
// espp::TaskMonitor) as a transport-agnostic dispatcher module
// (detail/monitor_protocol.hpp is the wire spec). Same contract as the other
// espp services: requests are handled under an internal mutex, the `send`
// callback runs with that mutex released, frames for other modules /
// reply-flagged frames are ignored so the service shares a stream.
//
// Wiring (one line per transport):
//
//   espp::MonitorService monitor_service({.send = [&](auto f) { usb.write_vendor(f); }});
//   dispatcher.register_module(monitor_service);   // module 8 + discovery metadata

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iterator>
#include <memory>
#include <mutex>
#include <span>
#include <string_view>
#include <vector>

#include "esp_heap_caps.h"

#include "dispatcher.hpp"
#include "stream_frame.hpp"

#include "base_component.hpp"
#include "detail/monitor_protocol.hpp"
#include "heap_monitor.hpp"
#include "task.hpp"
#include "task_monitor.hpp"

namespace espp {

/**
 * @brief Serves heap-region and task statistics over any framed byte stream
 *        (dispatcher module 8 by default; see Config::module).
 *
 * GET_HEAP answers with one record per configured heap region
 * (Config::heap_regions, MALLOC_CAP_* masks; regions that do not exist on the
 * chip, i.e. report a total size of 0, are left out). GET_TASKS answers with
 * espp::TaskMonitor's per-task table (name, CPU %, stack high-water mark,
 * priority, core); it needs CONFIG_FREERTOS_USE_TRACE_FACILITY and
 * CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS -- without them the list is empty
 * (logged once), never an error. A TASKS payload is capped at the frame
 * payload limit: tasks that do not fit are dropped from the end (logged).
 * SET_STREAM starts (or stops) a task that sends HEAP and / or TASKS events
 * periodically (period clamped to Config::min_stream_period), so a host can
 * plot them live without polling.
 *
 * **Threading**: an internal mutex covers the parser and request handling;
 * every outbound frame (replies and streamed events alike) is serialized on
 * a send mutex held across the `send` callback, so frames never interleave
 * and `send` never runs concurrently with itself. `send` must therefore not
 * re-enter this object.
 *
 * \section monitor_service_ex1 MonitorService Example
 * \snippet system_example.cpp system_example
 */
class MonitorService : public BaseComponent {
public:
  using Stream = espp::stream_frame::StreamParser;
  using Type = espp::detail::monitor_protocol::Type;

  /// Default dispatcher module id (8). A routing key only: Config::module serves
  /// on any id, and hosts find it through discovery (by kProtocol).
  static constexpr uint8_t kModule = espp::detail::monitor_protocol::kModule;
  /// Stable protocol identifier + version advertised through discovery.
  static constexpr const char *kProtocol = espp::detail::monitor_protocol::kProtocol;
  static constexpr uint16_t kProtocolVersion = espp::detail::monitor_protocol::kProtocolVersion;

  /// Transmits one encoded frame to the host.
  using send_fn = std::function<void(std::span<const uint8_t> frame)>;

  /// Configuration for the MonitorService.
  struct Config {
    send_fn send{nullptr}; ///< Transmits an encoded frame (required).
    /// Dispatcher module id this instance answers on (and stamps on every
    /// frame it sends). A routing key only (0x00..0xEF).
    uint8_t module{kModule};
    /// Heap regions reported, as MALLOC_CAP_* masks (HeapMonitor::get_info).
    std::vector<int> heap_regions{MALLOC_CAP_DEFAULT, MALLOC_CAP_INTERNAL, MALLOC_CAP_SPIRAM};
    /// Shortest streaming period a host may request (SET_STREAM is clamped to it).
    std::chrono::milliseconds min_stream_period{100};
    /// The streaming task (started on the first SET_STREAM enable).
    Task::BaseConfig task_config{.name = "monitor_stream", .stack_size_bytes = 6 * 1024};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /// @brief Construct the service.
  explicit MonitorService(const Config &config)
      : BaseComponent("MonitorService", config.log_level)
      , config_(config)
      , period_(config.min_stream_period) {}

  ~MonitorService() { stop_stream(); }

  /// @brief The dispatcher module id this service answers on (Config::module).
  uint8_t module_id() const { return config_.module; }

  /// @brief Discovery metadata for registering this service on a Dispatcher.
  Dispatcher::ModuleInfo module_info() const {
    return {.name = "Monitor",
            .app = "system_console.html",
            .description = "Heap regions and task statistics",
            .protocol = kProtocol,
            .protocol_version = kProtocolVersion};
  }

  /// @brief Whether periodic HEAP / TASKS events are being sent.
  bool streaming() const { return streaming_.load(); }

  /// @brief Stop streaming (also done by the destructor and by SET_STREAM 0).
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
   * @return true if the type belongs to the monitor protocol (a reply was
   *         sent), false if it was ignored.
   */
  bool handle_frame(uint8_t type, std::span<const uint8_t> payload) {
    namespace proto = espp::detail::monitor_protocol;
    switch (static_cast<Type>(type)) {
    case Type::GetHeap:
      send_frame(proto::build_frame(Type::Heap, build_heap(), module_id()));
      return true;
    case Type::GetTasks:
      send_frame(proto::build_frame(Type::Tasks, build_tasks(), module_id()));
      return true;
    case Type::SetStream: {
      const auto req = proto::decode_set_stream(payload);
      if (!req) {
        send_error(type, std::errc::invalid_argument,
                   "malformed SET_STREAM (expected u8 enable, u16 period_ms, u8 what)");
        return true;
      }
      if (req->enable && (req->what & (proto::kStreamHeap | proto::kStreamTasks)) == 0) {
        send_error(type, std::errc::invalid_argument, "SET_STREAM: nothing selected to stream");
        return true;
      }
      if (req->enable)
        start_stream(std::chrono::milliseconds(req->period_ms), req->what);
      else
        stop_stream();
      logger_.debug("SET_STREAM enable={} period_ms={} what=0x{:02x}", req->enable,
                    period_.load().count(), req->what);
      send_frame(proto::build_frame(Type::Ok, proto::encode_ok(type), module_id()));
      return true;
    }
    default:
      return false; // not a monitor request: ignore so the service can share a stream
    }
  }

protected:
  /// The HEAP payload for the configured regions (regions with no memory left out).
  std::vector<uint8_t> build_heap() const {
    namespace proto = espp::detail::monitor_protocol;
    std::vector<proto::HeapRegion> regions;
    regions.reserve(config_.heap_regions.size());
    for (const int flags : config_.heap_regions) {
      const HeapMonitor::HeapInfo hi = HeapMonitor::get_info(flags);
      if (hi.total_size == 0)
        continue; // e.g. MALLOC_CAP_SPIRAM on a chip without PSRAM
      regions.push_back({.flags = static_cast<uint32_t>(hi.heap_flags),
                         .free_bytes = static_cast<uint32_t>(hi.free_bytes),
                         .min_free_bytes = static_cast<uint32_t>(hi.min_free_bytes),
                         .largest_free_block = static_cast<uint32_t>(hi.largest_free_block),
                         .allocated_bytes = static_cast<uint32_t>(hi.allocated_bytes),
                         .total_size = static_cast<uint32_t>(hi.total_size)});
    }
    return proto::encode_heap(regions);
  }

  /// The TASKS payload (capped at the frame payload limit).
  std::vector<uint8_t> build_tasks() {
    namespace proto = espp::detail::monitor_protocol;
    const auto infos = TaskMonitor::get_latest_info_vector();
#if !(CONFIG_FREERTOS_USE_TRACE_FACILITY && CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS)
    logger_.warn_rate_limited("task statistics need CONFIG_FREERTOS_USE_TRACE_FACILITY and "
                              "CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS; reporting no tasks");
#endif
    std::vector<proto::TaskEntry> tasks;
    tasks.reserve(infos.size());
    // (without the FreeRTOS stats Kconfig `infos` is provably empty; the
    // conversion is still the right code for the configured build)
    // cppcheck-suppress knownEmptyContainer
    std::transform(
        infos.begin(), infos.end(), std::back_inserter(tasks), [](const TaskMonitor::TaskInfo &t) {
          return proto::TaskEntry{
              .name = t.name,
              .cpu_percent = static_cast<uint8_t>(t.cpu_percent > 100 ? 100 : t.cpu_percent),
              .high_water_mark = t.high_water_mark,
              .priority = static_cast<uint8_t>(t.priority > 255 ? 255 : t.priority),
              .core_id = static_cast<int8_t>(t.core_id)};
        });
    size_t encoded = 0;
    auto payload = proto::encode_tasks(tasks, espp::stream_frame::kMaxPayloadSize, &encoded);
    // cppcheck-suppress unsignedLessThanZero
    if (encoded < tasks.size())
      logger_.warn_rate_limited("TASKS payload full: reporting {} of {} tasks", encoded,
                                tasks.size());
    return payload;
  }

  void start_stream(std::chrono::milliseconds period, uint8_t what) {
    std::lock_guard<std::mutex> lock(mutex_);
    period_.store(std::max(period, config_.min_stream_period));
    what_.store(what);
    streaming_.store(true);
    if (task_)
      return; // already running: the new period / selection apply on its next wake
    task_ = std::make_unique<Task>(
        Task::Config{.callback = [this](std::mutex &m,
                                        std::condition_variable &cv) { return stream_step(m, cv); },
                     .task_config = config_.task_config});
    task_->start();
  }

  /// One streaming period: send the selected reports, then wait (interruptibly).
  bool stream_step(std::mutex &m, std::condition_variable &cv) {
    namespace proto = espp::detail::monitor_protocol;
    if (streaming_.load()) {
      const uint8_t what = what_.load();
      if (what & proto::kStreamHeap)
        send_frame(proto::build_frame(Type::Heap, build_heap(), module_id()));
      if (what & proto::kStreamTasks)
        send_frame(proto::build_frame(Type::Tasks, build_tasks(), module_id()));
    }
    std::unique_lock<std::mutex> lock(m);
    cv.wait_for(lock, period_.load());
    return false; // keep running until stopped
  }

  /// Transmit a frame. Serialized on send_mutex_ (held across the callback) so
  /// a streamed event and a reply never interleave. Never called with mutex_ held.
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

  void send_error(uint8_t request_type, std::errc errc, std::string_view message) {
    namespace proto = espp::detail::monitor_protocol;
    logger_.warn("{} (type 0x{:02x})", message, request_type);
    send_frame(proto::build_frame(
        Type::Error, proto::encode_error(request_type, static_cast<uint32_t>(errc), message),
        module_id()));
  }

private:
  Config config_;
  mutable std::mutex mutex_;      ///< guards the parser and task_
  mutable std::mutex send_mutex_; ///< serializes every outbound frame across `send`
  Stream parser_;
  std::unique_ptr<Task> task_;
  std::atomic<bool> streaming_{false};
  std::atomic<uint8_t> what_{0};
  std::atomic<std::chrono::milliseconds> period_;
};

// Compile-time check that the service keeps satisfying the dispatcher's module
// contract (module_id() / module_info() / handle(frame)).
static_assert(DispatcherModuleConcept<MonitorService>);

} // namespace espp
