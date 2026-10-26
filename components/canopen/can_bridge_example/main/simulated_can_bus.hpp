#pragma once

// A stand-in for espp::Twai that runs a SimulatedDs402Node instead of the TWAI
// peripheral (CONFIG_CAN_BRIDGE_SIMULATED_NODE). It exposes the subset of the
// espp::Twai interface the bridge uses -- Config (with the same field names),
// Message, initialize() / start() / stop() / transmit() -- so
// can_bridge_example.cpp selects it with one `using CanBus = ...` and is
// otherwise unchanged:
//
//   - transmit() hands the frame to the node as if it had been received on the
//     bus; the node's responses come back through Config::on_receive from the
//     bus task (like frames from the TWAI receive task), promptly but never
//     from inside transmit() itself. As with real hardware, a response and the
//     OK the bridge sends for the CAN_TX that caused it are not ordered with
//     respect to each other (the response may reach the host first); the web
//     apps match responses by COB-ID, not by their position after the OK.
//   - a bus task ticks the node (heartbeat, TPDO event timer, motion) every
//     Config::tick_period.
//   - in LISTEN_ONLY mode transmit() fails as a real listen-only node cannot
//     send; the node's own traffic (heartbeat, boot-up) is still observed.
//
// Frames the host sends are not echoed back (a controller does not receive its
// own transmissions).

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <system_error>
#include <vector>

#include "task.hpp"
#include "twai.hpp"

#include "simulated_ds402_node.hpp"

namespace can_bridge {

class SimulatedCanBus {
public:
  using Message = espp::Twai::Message;
  using Mode = espp::Twai::Mode;
  using receive_callback_fn = espp::Twai::receive_callback_fn;
  using error_callback_fn = espp::Twai::error_callback_fn;

  struct Config {
    int tx_gpio{-1};           ///< Ignored (kept for espp::Twai::Config parity).
    int rx_gpio{-1};           ///< Ignored.
    uint32_t baudrate{500000}; ///< Reported only; the simulation has no bit timing.
    Mode mode{Mode::NORMAL};   ///< LISTEN_ONLY refuses transmit().
    receive_callback_fn on_receive{
        nullptr}; ///< Called (in the bus task) for each frame the node sends.
    error_callback_fn on_error{nullptr}; ///< Never called (the simulated bus has no errors).
    bool auto_start{true};               ///< If true, the node runs at the end of initialize().
    uint8_t node_id{1};                  ///< The simulated node's CANopen node id (1..127).
    std::chrono::milliseconds tick_period{
        5}; ///< Simulation step (heartbeat / TPDO / motion); >= 1 ms.
  };

  explicit SimulatedCanBus(const Config &config)
      : config_(config)
      , node_(SimulatedDs402Node::Config{.node_id = config.node_id}) {}

  ~SimulatedCanBus() {
    std::error_code ec;
    stop(ec);
  }

  bool initialize(std::error_code &ec) {
    ec.clear();
    if (config_.node_id < 1 || config_.node_id > 127) {
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    if (config_.tick_period < std::chrono::milliseconds(1)) {
      // the bus task waits tick_period between steps: 0 would spin
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    initialized_ = true;
    return config_.auto_start ? start(ec) : true;
  }

  bool start(std::error_code &ec) {
    ec.clear();
    if (!initialized_) {
      ec = std::make_error_code(std::errc::operation_not_permitted);
      return false;
    }
    if (task_)
      return true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      node_.reset_communication(); // boot-up message on the first tick
      pending_.clear();
    }
    last_tick_ = std::chrono::steady_clock::now();
    task_ = std::make_unique<espp::Task>(
        espp::Task::Config{.callback = [this]() { return run(); },
                           .task_config = {.name = "sim_can_bus", .stack_size_bytes = 6 * 1024}});
    if (!task_->start()) {
      task_.reset(); // no bus task: transmit() must not accept frames nobody delivers
      ec = std::make_error_code(std::errc::resource_unavailable_try_again);
      return false;
    }
    return true;
  }

  bool stop(std::error_code &ec) {
    ec.clear();
    if (task_) {
      task_->stop(); // run() returns within one tick_period, so this is prompt
      task_.reset();
    }
    return true;
  }

  /// Hand a host frame to the simulated node. Its responses are delivered via
  /// Config::on_receive from the bus task (possibly before this returns).
  bool transmit(const Message &message, std::error_code &ec) {
    ec.clear();
    if (!task_) {
      ec = std::make_error_code(std::errc::not_connected);
      return false;
    }
    if (config_.mode == Mode::LISTEN_ONLY) {
      ec = std::make_error_code(
          std::errc::operation_not_permitted); // a listen-only node never transmits
      return false;
    }
    if (message.dlc > 8) {
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    espp::detail::CanFrame in;
    in.id = message.id;
    in.extended = message.extended;
    in.rtr = message.rtr;
    in.dlc = message.dlc;
    in.data = message.data;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      node_.process(in, pending_);
    }
    // wake the bus task so the reply goes out right away (it also runs every tick_period)
    {
      std::lock_guard<std::mutex> lock(wake_mutex_);
      wake_ = true;
    }
    wake_cv_.notify_one();
    return true;
  }

private:
  bool run() {
    {
      std::unique_lock<std::mutex> lock(wake_mutex_);
      wake_cv_.wait_for(lock, config_.tick_period, [this] { return wake_; });
      wake_ = false;
    }
    const auto now = std::chrono::steady_clock::now();
    const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tick_);
    // frames_ is reused across runs (cleared, then swapped with pending_) so
    // the periodic task does not allocate
    auto &frames = frames_;
    frames.clear();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (dt >= config_.tick_period) {
        last_tick_ = now;
        node_.tick(dt, pending_);
      }
      frames.swap(pending_);
    }
    // deliver outside the node lock: on_receive takes the bridge's stream / tx
    // mutexes and must never run with mutex_ held (transmit() holds it while
    // it feeds the node from a dispatcher worker)
    if (config_.on_receive) {
      for (const auto &f : frames) {
        Message m;
        m.id = f.id;
        m.extended = f.extended;
        m.rtr = f.rtr;
        m.dlc = f.dlc;
        m.data = f.data;
        config_.on_receive(m);
      }
    }
    return false; // keep running
  }

  Config config_;
  SimulatedDs402Node node_;
  std::mutex mutex_; // guards node_ + pending_
  std::vector<espp::detail::CanFrame> pending_;
  std::vector<espp::detail::CanFrame> frames_; // bus-task-only delivery buffer (reused)
  std::mutex wake_mutex_;                      // guards wake_ (transmit() -> bus task wake-up)
  std::condition_variable wake_cv_;
  bool wake_{false};
  bool initialized_{false};
  std::unique_ptr<espp::Task> task_;
  std::chrono::steady_clock::time_point last_tick_{};
};

} // namespace can_bridge
