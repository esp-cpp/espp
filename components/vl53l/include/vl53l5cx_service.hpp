#pragma once

// espp::Vl53l5cxService — the VL53L5CX console protocol (vl53l5cx_protocol.hpp)
// as a dispatcher service around an espp::Vl53l5cx driver running on the
// device: register it on an espp::Dispatcher / DispatcherWorker and a host (the
// hosted vl53l5cx_console.html web app) can configure the sensor, start / stop
// ranging and receive every ranging frame, with all the I2C work happening
// here.
//
//   espp::Vl53l5cx sensor({...});
//   std::mutex sensor_mutex; // one sensor: shared by every user of `sensor`
//   espp::Vl53l5cxService service(sensor, {.send = send, .sensor_mutex = &sensor_mutex});
//   link.register_module(service);
//   // the application's poll task, on every frame it reads:
//   service.on_frame(frame);
//
// The service does NOT poll the sensor itself: the application owns the one
// poll loop (there is only one sensor to read, however many transports serve
// it) and hands each frame to every service instance through on_frame(), which
// forwards it to the host when that transport asked for the frame stream.
// One instance per byte stream (like the other espp services); several
// instances (vendor + CDC) share one Vl53l5cx -- point them at the same
// `sensor_mutex` so their command sequences never interleave.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "base_component.hpp"
#include "dispatcher.hpp"
#include "stream_frame.hpp"
#include "vl53l5cx.hpp"
#include "vl53l5cx_protocol.hpp"

namespace espp {

/**
 * @brief Dispatcher service exposing an espp::Vl53l5cx over the VL53L5CX
 *        console protocol (module 10 by default).
 *
 * Requests are executed against the driver under the sensor mutex (a settings
 * write is a few DCI transactions, an INITIALIZE re-uploads the firmware for
 * about two seconds, so feed this service from a worker task, e.g. an
 * espp::DispatcherWorker) and the reply is sent after the mutex is released.
 * Frames arrive through on_frame() from the application's poll task and are
 * forwarded (every Nth, see SetFrameStream) while the host has the stream
 * enabled; the latest one is kept for GetFrame.
 *
 * \section vl53l5cx_service_ex1 Vl53l5cxService Example
 * \snippet vl53l5cx_webapp_example.cpp vl53l5cx_webapp_example
 */
class Vl53l5cxService : public BaseComponent {
public:
  using Request = vl53l5cx_protocol::Request;
  using Reply = vl53l5cx_protocol::Reply;
  using Status = vl53l5cx_protocol::Status;
  using Frame = vl53l5cx_protocol::Frame;

  /// Transmits one encoded frame to the host.
  using send_fn = std::function<void(std::span<const uint8_t> frame)>;

  /// Configuration for the Vl53l5cxService.
  struct Config {
    /// Transmits an encoded frame (required). Called from the task that feeds
    /// handle() (request replies) AND from the task that calls on_frame()
    /// (the frame stream), so it must be thread-safe: if the transport does
    /// not serialize writers (e.g. UsbDevice::write_vendor), guard it with a
    /// mutex.
    send_fn send{nullptr};
    /// Dispatcher module id to answer on (a routing key only: the console
    /// finds it through discovery, by kProtocol, whatever id is chosen).
    uint8_t module{vl53l5cx_protocol::kModuleId};
    /// Mutex serializing every use of the Vl53l5cx. Share it between all users
    /// of the driver -- other service instances and the application's poll
    /// loop. If null the service uses a private mutex.
    std::mutex *sensor_mutex{nullptr};
    /// Frame stream divider when the host asks for 0 (1 = every frame).
    uint8_t default_stream_divider{1};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /**
   * @brief Construct the service.
   * @param sensor The driver to expose (must outlive the service).
   * @param config Configuration parameters.
   */
  explicit Vl53l5cxService(Vl53l5cx &sensor, const Config &config)
      : BaseComponent("Vl53l5cxService", config.log_level)
      , sensor_(sensor)
      , config_(config)
      , sensor_mutex_(config.sensor_mutex ? *config.sensor_mutex : own_mutex_)
      , stream_divider_(config.default_stream_divider ? config.default_stream_divider : 1) {}

  Vl53l5cxService(const Vl53l5cxService &) = delete;
  Vl53l5cxService &operator=(const Vl53l5cxService &) = delete;

  /// @brief The dispatcher module id this service answers on.
  uint8_t module_id() const { return config_.module; }

  /// @brief Discovery metadata for registering this service on a Dispatcher.
  Dispatcher::ModuleInfo module_info() const {
    return {.name = "VL53L5CX",
            .app = "vl53l5cx_console.html",
            .description = "Configure a VL53L5CX / VL53L7CX multizone ToF sensor & stream frames",
            .protocol = vl53l5cx_protocol::kProtocol,
            .protocol_version = vl53l5cx_protocol::kProtocolVersion};
  }

  /// @brief The mutex serializing use of the driver (take it around your own
  ///        Vl53l5cx calls when sharing the driver with this service).
  std::mutex &sensor_mutex() { return sensor_mutex_; }

  /// @brief Dispatcher entry point: handle one routed frame. Frames for other
  ///        modules and reply-flagged frames (echoes) are ignored.
  void handle(const espp::stream_frame::Frame &frame) {
    if (frame.module != module_id() || frame.is_reply())
      return;
    handle_request(frame.type, frame.payload);
  }

  /**
   * @brief Hand the service a frame the application just read from the sensor.
   * @param data The frame (Vl53l5cx::get_ranging_data() output).
   * @note Call it from the poll task for EVERY frame, whether or not a host is
   *       streaming: it keeps the frame counter / rate (STATUS) and the latest
   *       frame (GetFrame), and forwards the frame on this transport when the
   *       stream is enabled (every `divider`-th frame). Must be called WITHOUT
   *       the sensor mutex held (it transmits).
   */
  void on_frame(const Vl53l5cx::RangingData &data) {
    const auto now = std::chrono::steady_clock::now();
    const uint32_t seq = ++frame_count_;
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      if (last_frame_time_.time_since_epoch().count() != 0) {
        const auto period =
            std::chrono::duration_cast<std::chrono::microseconds>(now - last_frame_time_).count();
        if (period > 0) {
          // exponential average of the period (a quarter of the way per frame)
          const float f = period;
          period_us_ = period_us_ > 0.0f ? period_us_ + (f - period_us_) * 0.25f : f;
        }
      }
      last_frame_time_ = now;
      latest_ = to_wire(data, seq, uptime_ms());
      have_latest_ = true;
      if (!stream_enabled_.load())
        return;
      const uint8_t divider = stream_divider_.load();
      if (divider > 1 && (seq % divider) != 0)
        return;
      pending_ = latest_.serialize();
    }
    // transmit with no lock held (the sender may block on USB backpressure)
    std::vector<uint8_t> payload;
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      payload.swap(pending_);
    }
    if (!payload.empty())
      send(Reply::Frame, payload);
  }

  /// @brief Take a STATUS snapshot (reads the settings back from the sensor
  ///        when it is idle; the cached values while it is ranging).
  Status read_status() {
    Status s;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      if (sensor_.is_initialized() && !sensor_.is_ranging())
        refresh_settings_locked();
      s = settings_;
      s.initialized = sensor_.is_initialized();
      s.ranging = sensor_.is_ranging();
      s.motion_enabled = sensor_.is_motion_indicator_enabled();
      s.variant = static_cast<uint8_t>(sensor_.variant());
      s.address = sensor_.address();
    }
    s.stream_enabled = stream_enabled_.load();
    s.stream_divider = stream_divider_.load();
    s.frame_count = frame_count_.load();
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      const bool fresh =
          last_frame_time_.time_since_epoch().count() != 0 &&
          std::chrono::steady_clock::now() - last_frame_time_ < std::chrono::seconds(2);
      s.frame_rate_chz = fresh && period_us_ > 0.0f
                             ? static_cast<uint16_t>(100.0f * 1.0e6f / period_us_ + 0.5f)
                             : 0;
    }
    return s;
  }

  /// @brief Send one STATUS snapshot now (what GET_STATUS does).
  void send_status() { send(Reply::Status, read_status().serialize()); }

  /// @brief Enable / disable the frame stream on this transport.
  /// @param divider Every divider-th frame is sent (0 = default).
  void set_frame_stream(bool enabled, uint8_t divider = 0) {
    stream_divider_.store(divider                          ? divider
                          : config_.default_stream_divider ? config_.default_stream_divider
                                                           : 1);
    stream_enabled_.store(enabled);
    logger_.info("frame stream {} (every {} frame(s))", enabled ? "enabled" : "disabled",
                 stream_divider_.load());
  }

  /// @brief Whether the frame stream is currently enabled.
  bool stream_enabled() const { return stream_enabled_.load(); }

protected:
  /// Convert a driver frame to the wire structure.
  static Frame to_wire(const Vl53l5cx::RangingData &data, uint32_t seq, uint32_t timestamp_ms) {
    Frame f;
    f.sequence = seq;
    f.timestamp_ms = timestamp_ms;
    f.zones = static_cast<uint8_t>(data.num_zones());
    for (size_t i = 0; i < data.num_zones() && i < f.zone.size(); i++) {
      auto &z = f.zone[i];
      z.distance_mm = data.distance_mm[i];
      z.status = data.target_status[i];
      z.nb_targets = data.nb_target_detected[i];
      z.reflectance_pct = data.reflectance_percent[i];
      z.sigma_mm = data.range_sigma_mm[i];
      z.signal_kcps = data.signal_per_spad_kcps[i];
      z.ambient_kcps = data.ambient_per_spad_kcps[i];
      z.spads = data.nb_spads_enabled[i];
    }
    const auto &m = data.motion_indicator;
    // the sensor fills the block only when the indicator is enabled: the
    // aggregate count is then non-zero
    f.motion_present = m.nb_of_aggregates != 0;
    f.motion.global_indicator_1 = m.global_indicator_1;
    f.motion.global_indicator_2 = m.global_indicator_2;
    f.motion.status = m.status;
    f.motion.nb_of_detected_aggregates = m.nb_of_detected_aggregates;
    f.motion.nb_of_aggregates = m.nb_of_aggregates;
    f.motion.spare = m.spare;
    f.motion.motion = m.motion;
    return f;
  }

  static uint32_t uptime_ms() {
    static const auto t0 = std::chrono::steady_clock::now();
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
            .count());
  }

  /// Read every setting back from the sensor (sensor mutex held, sensor idle).
  void refresh_settings_locked() {
    std::error_code ec;
    settings_.zones = static_cast<uint8_t>(sensor_.get_resolution(ec));
    settings_.frequency_hz = sensor_.get_ranging_frequency_hz(ec);
    settings_.integration_ms = sensor_.get_integration_time_ms(ec);
    settings_.sharpener = sensor_.get_sharpener_percent(ec);
    settings_.target_order = static_cast<uint8_t>(sensor_.get_target_order(ec));
    settings_.ranging_mode = static_cast<uint8_t>(sensor_.get_ranging_mode(ec));
    settings_.power_mode = static_cast<uint8_t>(sensor_.get_power_mode(ec));
    if (ec)
      logger_.warn("reading the settings back failed: {}", ec.message());
  }

  /// Execute one request against the driver (under the sensor mutex) and
  /// reply after the mutex is released.
  void handle_request(uint8_t type, std::span<const uint8_t> payload) {
    namespace proto = vl53l5cx_protocol;
    using Resolution = Vl53l5cx::Resolution;
    using TargetOrder = Vl53l5cx::TargetOrder;
    using RangingMode = Vl53l5cx::RangingMode;
    using PowerMode = Vl53l5cx::PowerMode;
    switch (static_cast<Request>(type)) {
    case Request::GetStatus:
      send_status();
      return;
    case Request::SetFrameStream: {
      const auto req = proto::SetFrameStream::parse(payload);
      if (!req) {
        send_error(type, std::errc::invalid_argument, "short payload");
        return;
      }
      set_frame_stream(req->enabled, req->divider);
      send_ok(type);
      return;
    }
    case Request::GetFrame: {
      std::vector<uint8_t> frame;
      {
        std::lock_guard<std::mutex> lock(latest_mutex_);
        if (have_latest_)
          frame = latest_.serialize();
      }
      if (frame.empty())
        send_error(type, std::errc::no_message_available, "no frame read yet (start ranging)");
      else
        send(Reply::Frame, frame);
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
      std::lock_guard<std::mutex> lock(sensor_mutex_);
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
      const auto arg = proto::ByteArg::parse(payload);
      switch (static_cast<Request>(type)) {
      case Request::SetResolution:
        if (arg && (arg->value == 16 || arg->value == 64)) {
          const auto r = arg->value == 64 ? Resolution::ZONES_8X8 : Resolution::ZONES_4X4;
          result(sensor_.set_resolution(r, ec), "set resolution failed");
          if (!ec)
            settings_.zones = arg->value;
        } else {
          malformed("resolution must be 16 (4x4) or 64 (8x8)");
        }
        break;
      case Request::SetRangingFrequency:
        if (arg) {
          result(sensor_.set_ranging_frequency_hz(arg->value, ec), "set ranging frequency failed");
          if (!ec)
            settings_.frequency_hz = arg->value;
        } else {
          malformed("short payload");
        }
        break;
      case Request::SetIntegrationTime:
        if (const auto r = proto::SetIntegrationTime::parse(payload)) {
          result(sensor_.set_integration_time_ms(r->time_ms, ec), "set integration time failed");
          if (!ec)
            settings_.integration_ms = r->time_ms;
        } else {
          malformed("short payload");
        }
        break;
      case Request::SetSharpener:
        if (arg) {
          result(sensor_.set_sharpener_percent(arg->value, ec), "set sharpener failed");
          if (!ec)
            settings_.sharpener = arg->value;
        } else {
          malformed("short payload");
        }
        break;
      case Request::SetTargetOrder:
        if (arg && (arg->value == static_cast<uint8_t>(TargetOrder::CLOSEST) ||
                    arg->value == static_cast<uint8_t>(TargetOrder::STRONGEST))) {
          result(sensor_.set_target_order(static_cast<TargetOrder>(arg->value), ec),
                 "set target order failed");
          if (!ec)
            settings_.target_order = arg->value;
        } else {
          malformed("target order must be 1 (closest) or 2 (strongest)");
        }
        break;
      case Request::SetRangingMode:
        if (arg && (arg->value == static_cast<uint8_t>(RangingMode::CONTINUOUS) ||
                    arg->value == static_cast<uint8_t>(RangingMode::AUTONOMOUS))) {
          result(sensor_.set_ranging_mode(static_cast<RangingMode>(arg->value), ec),
                 "set ranging mode failed");
          if (!ec)
            settings_.ranging_mode = arg->value;
        } else {
          malformed("ranging mode must be 1 (continuous) or 3 (autonomous)");
        }
        break;
      case Request::SetPowerMode:
        if (arg && (arg->value == static_cast<uint8_t>(PowerMode::SLEEP) ||
                    arg->value == static_cast<uint8_t>(PowerMode::WAKEUP))) {
          result(sensor_.set_power_mode(static_cast<PowerMode>(arg->value), ec),
                 "set power mode failed");
          if (!ec)
            settings_.power_mode = arg->value;
        } else {
          malformed("power mode must be 0 (sleep) or 1 (wakeup)");
        }
        break;
      case Request::EnableMotionIndicator:
        if (const auto r = proto::EnableMotionIndicator::parse(payload)) {
          result(sensor_.enable_motion_indicator(r->min_mm, r->max_mm, ec),
                 "enable motion indicator failed");
          if (!ec) {
            settings_.motion_min_mm = r->min_mm;
            settings_.motion_max_mm = r->max_mm;
          }
        } else {
          malformed("short payload");
        }
        break;
      case Request::StartRanging:
        result(sensor_.start_ranging(ec), "start ranging failed");
        break;
      case Request::StopRanging:
        result(sensor_.stop_ranging(ec), "stop ranging failed");
        break;
      case Request::Initialize:
        result(sensor_.initialize(ec), "initialize failed");
        settings_.motion_min_mm = settings_.motion_max_mm = 0; // the indicator is off again
        if (!ec)
          refresh_settings_locked();
        break;
      case Request::SetI2cAddress:
        if (arg)
          result(sensor_.set_i2c_address(arg->value, ec), "set I2C address failed");
        else
          malformed("short payload");
        break;
      default:
        reply_type = Reply::Error;
        reply = error_payload(type, std::make_error_code(std::errc::not_supported),
                              "unknown VL53L5CX message");
        break;
      }
    }
    send(reply_type, reply);
  }

  std::vector<uint8_t> error_payload(uint8_t request_type, const std::error_code &ec,
                                     std::string_view context) const {
    logger_.warn("{}: {}", context, ec.message());
    return vl53l5cx_protocol::make_error_payload(request_type, static_cast<uint32_t>(ec.value()),
                                                 std::string(context) + ": " + ec.message());
  }

  void send_ok(uint8_t request_type) {
    send(Reply::Ok, vl53l5cx_protocol::make_ok_payload(request_type));
  }

  void send_error(uint8_t request_type, std::errc errc, std::string_view context) {
    send(Reply::Error, error_payload(request_type, std::make_error_code(errc), context));
  }

  /// Encode and transmit one frame (reply flag from the type's high bit).
  /// Must be called WITHOUT the sensor mutex held.
  void send(Reply type, std::span<const uint8_t> payload) {
    if (!config_.send) {
      logger_.warn("no send function configured; dropping a reply");
      return;
    }
    const auto t = static_cast<uint8_t>(type);
    config_.send(
        espp::stream_frame::build_frame(vl53l5cx_protocol::is_reply(t), module_id(), t, payload));
  }

  Vl53l5cx &sensor_;
  Config config_;
  std::mutex own_mutex_;
  std::mutex &sensor_mutex_;
  Status settings_{}; ///< last settings read from / written to the sensor (sensor mutex)
  std::atomic<bool> stream_enabled_{false};
  std::atomic<uint8_t> stream_divider_;
  std::atomic<uint32_t> frame_count_{0};
  std::mutex latest_mutex_; ///< guards latest_ / have_latest_ / pending_ / the rate estimate
  Frame latest_{};
  bool have_latest_{false};
  std::vector<uint8_t> pending_;
  std::chrono::steady_clock::time_point last_frame_time_{};
  float period_us_{0.0f};
};

// Compile-time check that the service keeps satisfying the dispatcher's module
// contract (module_id() / module_info() / handle(frame)) -- see
// espp::DispatcherModuleConcept.
static_assert(DispatcherModuleConcept<Vl53l5cxService>);

} // namespace espp
