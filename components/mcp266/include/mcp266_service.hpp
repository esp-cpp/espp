#pragma once

// espp::Mcp266Service — the MCP266 console protocol (mcp266_protocol.hpp) as
// a dispatcher service around an espp::Mcp266 driver running on the device:
// register it on an espp::Dispatcher / DispatcherWorker and a host (the hosted
// mcp266_console.html web app) can configure the position loops, command
// moves, read live status and stream it, with all the CANopen/DS402 work
// happening here.
//
//   espp::Mcp266 mcp(client, {...});
//   std::mutex mcp_mutex; // one SDO channel: shared by every user of `mcp`
//   espp::Mcp266Service service(mcp, {.send = send, .mcp_mutex = &mcp_mutex});
//   link.register_module(service);
//
// One instance per byte stream (like the other espp services): its STATUS
// stream, when the host enables it, goes out on that stream's `send`. Several
// instances (vendor + CDC) may share one Mcp266 -- point them at the same
// `mcp_mutex` so their SDO transactions never interleave.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "base_component.hpp"
#include "dispatcher.hpp"
#include "mcp266.hpp"
#include "mcp266_protocol.hpp"
#include "stream_frame.hpp"
#include "task.hpp"

namespace espp {

/**
 * @brief Dispatcher service exposing an espp::Mcp266 over the MCP266 console
 *        protocol (module 6 by default).
 *
 * Requests are executed against the driver under the MCP mutex (SDO
 * transactions block, so feed this service from a worker task, e.g. an
 * espp::DispatcherWorker) and the reply is sent after the mutex is released.
 * A STATUS snapshot is eight SDO reads; the host-requested stream period is
 * clamped to [Config::min_stream_period_ms, Config::max_stream_period_ms] so
 * the stream can neither starve command handling nor flood the CAN bus.
 *
 * \section mcp266_service_ex1 Mcp266Service Example
 * \snippet mcp266_webapp_example.cpp mcp266_webapp_example
 */
class Mcp266Service : public BaseComponent {
public:
  using Axis = Mcp266::Axis;
  using Request = mcp266_protocol::Request;
  using Reply = mcp266_protocol::Reply;
  using Status = mcp266_protocol::Status;

  /// Transmits one encoded frame to the host.
  using send_fn = std::function<void(std::span<const uint8_t> frame)>;

  /// Configuration for the Mcp266Service.
  struct Config {
    send_fn send{nullptr}; ///< Transmits an encoded frame (required).
    /// Dispatcher module id to answer on (the console expects the default).
    uint8_t module{mcp266_protocol::kModuleId};
    /// Mutex serializing every use of the Mcp266 (one SDO channel). Share it
    /// between all users of the driver -- other service instances and the
    /// application's own calls. If null the service uses a private mutex.
    std::mutex *mcp_mutex{nullptr};
    uint16_t default_stream_period_ms{200}; ///< STATUS period when the host asks for 0.
    uint16_t min_stream_period_ms{50};      ///< Fastest STATUS stream (8 SDO reads each).
    uint16_t max_stream_period_ms{10000};   ///< Slowest STATUS stream.
    /// The status-streaming task (one per service instance).
    Task::BaseConfig status_task_config{.name = "mcp266_status", .stack_size_bytes = 8192};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /**
   * @brief Construct the service (and start its status-streaming task, idle
   *        until a host enables the stream).
   * @param mcp The driver to expose (must outlive the service).
   * @param config Configuration parameters.
   */
  explicit Mcp266Service(Mcp266 &mcp, const Config &config)
      : BaseComponent("Mcp266Service", config.log_level)
      , mcp_(mcp)
      , config_(config)
      , mcp_mutex_(config.mcp_mutex ? *config.mcp_mutex : own_mutex_)
      , stream_period_ms_(config.default_stream_period_ms) {
    status_task_ = Task::make_unique(
        {.callback = [this](std::mutex &m,
                            std::condition_variable &cv) { return status_task_fn(m, cv); },
         .task_config = config.status_task_config});
    status_task_->start();
  }

  ~Mcp266Service() {
    if (status_task_)
      status_task_->stop();
  }

  Mcp266Service(const Mcp266Service &) = delete;
  Mcp266Service &operator=(const Mcp266Service &) = delete;

  /// @brief The dispatcher module id this service answers on.
  uint8_t module_id() const { return config_.module; }

  /// @brief Discovery metadata for registering this service on a Dispatcher.
  Dispatcher::ModuleInfo module_info() const {
    return {.name = "MCP266",
            .app = "mcp266_console.html",
            .description = "Configure & command MCP266 motors"};
  }

  /// @brief The mutex serializing use of the driver (take it around your own
  ///        Mcp266 calls when sharing the driver with this service).
  std::mutex &mcp_mutex() { return mcp_mutex_; }

  /// @brief Dispatcher entry point: handle one routed frame. Frames for other
  ///        modules and reply-flagged frames (echoes) are ignored.
  void handle(const espp::stream_frame::Frame &frame) {
    if (frame.module != module_id() || frame.is_reply())
      return;
    handle_request(frame.type, frame.payload);
  }

  /// @brief Take a STATUS snapshot (eight SDO reads under the MCP mutex).
  Status read_status() {
    Status s;
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    std::error_code ec;
    bool any_ok = false;
    for (auto [axis, out] : {std::pair{Axis::M1, &s.m1}, std::pair{Axis::M2, &s.m2}}) {
      if (mcp_.read_encoder(axis, out->position, ec))
        any_ok = true;
      mcp_.read_speed(axis, out->velocity, ec);
      mcp_.read_statusword(axis, out->statusword, ec);
    }
    float volts = 0.0f, temp_c = 0.0f;
    mcp_.read_main_battery_voltage(volts, ec);
    mcp_.read_temperature(temp_c, ec);
    s.battery_decivolts = static_cast<uint16_t>(volts * 10.0f + 0.5f);
    s.temp_decidegrees = static_cast<uint16_t>(temp_c * 10.0f + 0.5f);
    s.online = any_ok;
    return s;
  }

  /// @brief Send one STATUS snapshot now (what GET_STATUS and the stream do).
  void send_status() { send(Reply::Status, read_status().serialize()); }

  /// @brief Enable / disable the periodic STATUS stream on this transport.
  /// @param period_ms Requested period (0 = default); clamped to the configured range.
  void set_status_stream(bool enabled, uint16_t period_ms = 0) {
    const uint16_t requested = period_ms == 0 ? config_.default_stream_period_ms : period_ms;
    stream_period_ms_.store(
        std::clamp(requested, config_.min_stream_period_ms, config_.max_stream_period_ms));
    stream_enabled_.store(enabled);
    logger_.info("status stream {} ({} ms)", enabled ? "enabled" : "disabled",
                 stream_period_ms_.load());
  }

  /// @brief Whether the STATUS stream is currently enabled.
  bool stream_enabled() const { return stream_enabled_.load(); }
  /// @brief The current STATUS stream period.
  uint16_t stream_period_ms() const { return stream_period_ms_.load(); }

protected:
  /// Execute one request against the driver (under the MCP mutex) and reply
  /// after the mutex is released.
  void handle_request(uint8_t type, std::span<const uint8_t> payload) {
    namespace proto = mcp266_protocol;
    // GET_STATUS / SET_STATUS_STREAM don't need the request lock held across
    // a driver call in the same way; handle them first.
    switch (static_cast<Request>(type)) {
    case Request::GetStatus:
      send_status();
      return;
    case Request::SetStatusStream: {
      const auto req = proto::SetStatusStream::parse(payload);
      if (!req) {
        send_error(type, std::errc::invalid_argument, "short payload");
        return;
      }
      set_status_stream(req->enabled, req->period_ms);
      send_ok(type);
      return;
    }
    default:
      break;
    }

    // Everything else is a driver call: run it under the mutex, build the
    // reply, then send with the mutex released.
    std::vector<uint8_t> reply;
    Reply reply_type = Reply::Ok;
    {
      std::lock_guard<std::mutex> lock(mcp_mutex_);
      std::error_code ec;
      auto result = [&](bool ok, const char *context) {
        if (ok) {
          reply = proto::make_ok_payload(type);
        } else {
          reply_type = Reply::Error;
          reply = error_payload(type, ec, context);
        }
      };
      auto malformed = [&](const char *what) {
        reply_type = Reply::Error;
        reply = error_payload(type, std::make_error_code(std::errc::invalid_argument), what);
      };
      switch (static_cast<Request>(type)) {
      case Request::Start:
        result(mcp_.start(ec), "start failed");
        break;
      case Request::ResetFaults:
        result(mcp_.reset_faults(ec), "reset faults failed");
        break;
      case Request::ResetEstop:
        result(mcp_.reset_estop(ec), "reset e-stop failed");
        break;
      case Request::ConfigurePositionLoop:
        if (const auto r = proto::ConfigurePositionLoop::parse(payload))
          result(mcp_.configure_position_loop(to_axis(r->axis), r->min, r->max, r->fallback_p, ec),
                 "configure position loop failed");
        else
          malformed("short payload or invalid axis (must be 0=M1 or 1=M2)");
        break;
      case Request::SetPositionLimits:
        if (const auto r = proto::SetPositionLimits::parse(payload))
          result(mcp_.set_software_position_limits(to_axis(r->axis), r->min, r->max, ec),
                 "set position limits failed");
        else
          malformed("short payload or invalid axis (must be 0=M1 or 1=M2)");
        break;
      case Request::MoveToPosition:
        if (const auto r = proto::MoveToPosition::parse(payload))
          result(mcp_.move_to_position(to_axis(r->axis), r->target, r->velocity, r->accel, r->decel,
                                       ec),
                 "move failed");
        else
          malformed("short payload or invalid axis (must be 0=M1 or 1=M2)");
        break;
      case Request::DriveSpeed:
        if (const auto r = proto::DriveSpeed::parse(payload))
          result(mcp_.drive_speed(to_axis(r->axis), r->qpps, ec), "drive speed failed");
        else
          malformed("short payload or invalid axis (must be 0=M1 or 1=M2)");
        break;
      case Request::DriveDuty:
        if (const auto r = proto::DriveDuty::parse(payload))
          result(mcp_.drive_duty(to_axis(r->axis), r->duty, ec), "drive duty failed");
        else
          malformed("short payload or invalid axis (must be 0=M1 or 1=M2)");
        break;
      case Request::GetDeviceInfo: {
        proto::DeviceInfo info;
        if (mcp_.read_device_info(info.name, info.device_type, ec)) {
          reply_type = Reply::DeviceInfo;
          reply = info.serialize();
        } else {
          reply_type = Reply::Error;
          reply = error_payload(type, ec, "read device info failed");
        }
        break;
      }
      default:
        reply_type = Reply::Error;
        reply = error_payload(type, std::make_error_code(std::errc::not_supported),
                              "unknown MCP266 message");
        break;
      }
    }
    send(reply_type, reply);
  }

  static Axis to_axis(mcp266_protocol::Axis a) {
    return a == mcp266_protocol::Axis::M2 ? Axis::M2 : Axis::M1;
  }

  bool status_task_fn(std::mutex &m, std::condition_variable &cv) {
    if (stream_enabled_.load())
      send_status();
    std::unique_lock<std::mutex> lock(m);
    cv.wait_for(lock,
                std::chrono::milliseconds(stream_enabled_.load() ? stream_period_ms_.load() : 200));
    return false; // keep running
  }

  std::vector<uint8_t> error_payload(uint8_t request_type, const std::error_code &ec,
                                     std::string_view context) const {
    logger_.warn("{}: {}", context, ec.message());
    return mcp266_protocol::make_error_payload(request_type, static_cast<uint32_t>(ec.value()),
                                               std::string(context) + ": " + ec.message());
  }

  void send_ok(uint8_t request_type) {
    send(Reply::Ok, mcp266_protocol::make_ok_payload(request_type));
  }

  void send_error(uint8_t request_type, std::errc errc, std::string_view context) {
    send(Reply::Error, error_payload(request_type, std::make_error_code(errc), context));
  }

  /// Encode and transmit one frame (reply flag from the type's high bit).
  /// Must be called WITHOUT the MCP mutex held.
  void send(Reply type, std::span<const uint8_t> payload) {
    if (!config_.send) {
      logger_.warn("no send function configured; dropping a reply");
      return;
    }
    const auto t = static_cast<uint8_t>(type);
    config_.send(
        espp::stream_frame::build_frame(mcp266_protocol::is_reply(t), module_id(), t, payload));
  }

  Mcp266 &mcp_;
  Config config_;
  std::mutex own_mutex_;
  std::mutex &mcp_mutex_;
  std::atomic<bool> stream_enabled_{false};
  std::atomic<uint16_t> stream_period_ms_;
  std::unique_ptr<Task> status_task_;
};

} // namespace espp
