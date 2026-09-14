#pragma once

// espp::DispatcherWorker — a Dispatcher fed from its own worker task.
//
// Transports deliver received bytes from contexts that must not block: the
// TinyUSB task (USB vendor / CDC receive callbacks), a socket reactor, a UART
// ISR-fed task, ... while protocol handlers routinely DO block (an OTA BEGIN
// erases a partition, a core-dump ERASE takes tens of milliseconds, a CAN
// bridge waits on a bus). Every espp example used to bridge the two with the
// same hand-written bounded queue + espp::Task + overflow bookkeeping. This
// class is that bridge:
//
//   espp::DispatcherWorker link({.send = [&](auto f) { usb.write_vendor(f); },
//                                .on_overflow = [&] { ota_service.on_rx_overflow(); },
//                                .task_config = {.name = "usb_rx", .stack_size_bytes = 8192}});
//   link.register_module(ota_service);            // any espp protocol service
//   link.register_module(coredump_service);
//   link.serve_discovery("My Device", version);   // module 0xFF, replies via `send`
//   usb.set_vendor_receive_callback([&](auto data) { link.push(data); });
//
// push() copies the bytes into a bounded queue and wakes the worker; the worker
// feeds the Dispatcher, which routes each frame to its module handler -- so
// handlers (and their `send` callbacks) always run on the worker task, never
// on the transport's. One DispatcherWorker per byte stream: a device exposing
// the same protocols over USB vendor AND USB CDC creates two (each with its
// own `send`), exactly like one Dispatcher per stream.
//
// Overflow: the queue is bounded (`max_queued_bytes`, default 8 max-size
// frames -- the espp protocols are one-request-in-flight, so a well-behaved
// peer never queues more than ~one). When a push would exceed the bound,
// everything queued is dropped (a partial frame is useless once bytes are
// missing), the Dispatcher's parser is reset so it resynchronizes on the next
// frame magic, and `on_overflow` runs on the worker so protocols can abort an
// in-flight transfer and tell the peer (e.g. OtaService::on_rx_overflow()).
//
// Transport lifecycle: call request_reset() from a mount / unmount callback to
// have the worker discard any half-parsed frame before the next bytes -- the
// parser is only ever touched from the worker, so this is safe from any task.

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "base_component.hpp"
#include "dispatcher.hpp"
#include "stream_frame.hpp"
#include "task.hpp"

namespace espp {

/// @brief A Dispatcher plus the bounded receive queue and worker task that feed it.
class DispatcherWorker : public BaseComponent {
public:
  /// Transmit callback: sends one encoded frame over this worker's transport.
  using send_fn = Dispatcher::reply_fn;
  /// Called on the worker task after an RX overflow was handled (queue
  /// cleared, parser reset).
  using overflow_fn = std::function<void()>;

  /// Configuration for the DispatcherWorker.
  struct Config {
    /// Sends one encoded frame over this transport. Used by serve_discovery()
    /// and returned by sender() so services can share it; optional if the
    /// worker only routes.
    send_fn send{nullptr};
    /// Notified on the worker task after an overflow (see the header comment).
    overflow_fn on_overflow{nullptr};
    /// Bound on queued-but-unprocessed bytes; a push that would exceed it
    /// drops everything queued and flags an overflow.
    size_t max_queued_bytes{8 * stream_frame::kMaxFrameSize};
    /// Worker task settings (name / stack / priority / core). Size the stack
    /// for the handlers it runs (OTA + logging comfortably fit 8 KiB).
    Task::BaseConfig task_config{.name = "dispatcher", .stack_size_bytes = 8192};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /// @brief Construct and start the worker task.
  explicit DispatcherWorker(const Config &config)
      : BaseComponent("DispatcherWorker", config.log_level)
      , send_(config.send)
      , on_overflow_(config.on_overflow)
      , max_queued_bytes_(config.max_queued_bytes) {
    task_ = Task::make_unique(
        {.callback = [this](std::mutex &m, std::condition_variable &cv) { return worker(m, cv); },
         .task_config = config.task_config});
    task_->start();
  }

  /// @brief Stop the worker task (queued bytes are discarded).
  ~DispatcherWorker() {
    {
      std::lock_guard<std::mutex> lock(rx_mutex_);
      running_ = false; // read by the worker under the same mutex
    }
    rx_cv_.notify_all();
    // Task::stop() flags the task and joins it; the worker returns from its
    // (bounded) wait promptly once running_ is false.
    if (task_)
      task_->stop();
  }

  DispatcherWorker(const DispatcherWorker &) = delete;
  DispatcherWorker &operator=(const DispatcherWorker &) = delete;

  /// @brief The transmit function configured for this transport (for services
  ///        that reply on the stream they were registered on).
  send_fn sender() const { return send_; }

  // --- registration (thread-safe with respect to the worker) -------------------

  /// @brief Register a protocol service satisfying DispatcherModuleConcept
  ///        (see Dispatcher::register_module(Service&)).
  template <DispatcherModuleConcept Service> void register_module(Service &service) {
    std::lock_guard<std::mutex> lock(dispatcher_mutex_);
    dispatcher_.register_module(service);
  }

  /// @brief Register a raw handler for a module id (see Dispatcher::register_module).
  void register_module(uint8_t module_id, Dispatcher::handler_fn handler,
                       Dispatcher::ModuleInfo info = {}) {
    std::lock_guard<std::mutex> lock(dispatcher_mutex_);
    dispatcher_.register_module(module_id, std::move(handler), std::move(info));
  }

  /// @brief Remove a module's handler (see Dispatcher::unregister_module).
  void unregister_module(uint8_t module_id) {
    std::lock_guard<std::mutex> lock(dispatcher_mutex_);
    dispatcher_.unregister_module(module_id);
  }

  /// @brief Advertise the device and answer capability discovery (module 0xFF)
  ///        over this transport's `send` (see Dispatcher::serve_discovery).
  /// @param device_name Device name shown by discovery peers.
  /// @param firmware Firmware version string (optional).
  void serve_discovery(std::string device_name, std::string firmware = "") {
    std::lock_guard<std::mutex> lock(dispatcher_mutex_);
    dispatcher_.set_device_info(std::move(device_name), std::move(firmware));
    dispatcher_.serve_discovery(send_);
  }

  /// @brief Direct access to the underlying Dispatcher. Only safe from a module
  ///        handler (which runs on the worker) or before any bytes are pushed;
  ///        prefer the register_module()/serve_discovery() wrappers otherwise.
  Dispatcher &dispatcher() { return dispatcher_; }

  // --- receive path ---------------------------------------------------------------

  /// @brief Queue received bytes for the worker. Callable from any task (e.g.
  ///        a transport's receive callback); never blocks beyond a short lock.
  /// @return false if the bytes were dropped (overflow).
  bool push(std::span<const uint8_t> data) {
    if (data.empty())
      return true;
    bool accepted = true;
    {
      std::lock_guard<std::mutex> lock(rx_mutex_);
      if (rx_queued_bytes_ + data.size() > max_queued_bytes_) {
        // partial frames are useless once bytes are missing: drop everything
        // and let the worker resynchronize + notify
        rx_queue_.clear();
        rx_queued_bytes_ = 0;
        rx_overflow_ = true;
        accepted = false;
      } else {
        rx_queue_.emplace_back(data.begin(), data.end());
        rx_queued_bytes_ += data.size();
      }
    }
    rx_cv_.notify_one();
    return accepted;
  }

  /// @brief Ask the worker to discard any half-parsed frame before the next
  ///        bytes (transport connect / disconnect). Bytes still queued from
  ///        before the reset are dropped too -- they belong to the old link
  ///        and must not be fed after the parser reset. Safe from any task.
  void request_reset() {
    {
      std::lock_guard<std::mutex> lock(rx_mutex_);
      rx_queue_.clear();
      rx_queued_bytes_ = 0;
      rx_reset_ = true;
    }
    rx_cv_.notify_one();
  }

  /// @brief Bytes currently queued for the worker.
  size_t queued_bytes() const {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    return rx_queued_bytes_;
  }

  /// @brief Total overflow events (diagnostics).
  size_t overflows() const {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    return overflow_count_;
  }

protected:
  bool worker(std::mutex &, std::condition_variable &) {
    std::deque<std::vector<uint8_t>> chunks;
    bool overflowed = false, reset = false;
    {
      std::unique_lock<std::mutex> lock(rx_mutex_);
      rx_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
        return !rx_queue_.empty() || rx_overflow_ || rx_reset_ || !running_;
      });
      if (!running_)
        return false; // shutting down: do nothing; the destructor's Task::stop() ends the task
      std::swap(chunks, rx_queue_);
      rx_queued_bytes_ = 0;
      overflowed = rx_overflow_;
      reset = rx_reset_;
      rx_overflow_ = rx_reset_ = false;
      if (overflowed)
        ++overflow_count_;
    }
    if (overflowed || reset) {
      // a frame straddling the gap (or the reconnect) must not be completed
      // with unrelated later bytes: resynchronize on the next frame magic
      std::lock_guard<std::mutex> lock(dispatcher_mutex_);
      dispatcher_.reset();
    }
    if (overflowed) {
      logger_.warn("RX overflow: {} chunk(s) dropped", chunks.size());
      chunks.clear(); // whatever was queued is gone with the dropped bytes
      if (on_overflow_)
        on_overflow_();
    }
    for (const auto &chunk : chunks) {
      std::lock_guard<std::mutex> lock(dispatcher_mutex_);
      dispatcher_.feed(chunk);
    }
    return false; // keep running
  }

  send_fn send_;
  overflow_fn on_overflow_;
  size_t max_queued_bytes_;

  std::mutex dispatcher_mutex_; // registration vs. feed/reset on the worker
  Dispatcher dispatcher_;

  mutable std::mutex rx_mutex_;
  std::condition_variable rx_cv_;
  std::deque<std::vector<uint8_t>> rx_queue_;
  size_t rx_queued_bytes_{0};
  bool rx_overflow_{false};
  bool rx_reset_{false};
  size_t overflow_count_{0};
  bool running_{true};
  std::unique_ptr<Task> task_;
};

} // namespace espp
