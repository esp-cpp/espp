// Hand-written pybind11 bindings for espp::RtpsParticipant (the facade over the
// embeddedRTPS engine in components/rtps — see its REFACTOR_PLAN.md).
//
// Why hand-written (like cdr): the participant exposes std::function callbacks
// taking std::span<const uint8_t> (no pybind caster) and is invoked from engine
// background threads, so callbacks must be wrapped GIL-correctly. This shim
// exposes a clean Python API:
//   - RtpsParticipant(Config(interface_address=..., ...))
//   - add_writer(topic=..., type_name=..., reliable=...)
//   - add_reader(topic=..., type_name=..., reliable=..., on_sample=callable(bytes))
//   - publish(topic, bytes)
//
// It is kept out of the generated pybind_espp.cpp so regeneration never clobbers it.

#include <algorithm>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <chrono>
#include <optional>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "rtps_participant.hpp"
#include "task.hpp"

namespace py = pybind11;
using Rtps = espp::RtpsParticipant;

namespace {

py::bytes to_bytes(std::span<const uint8_t> s) {
  return py::bytes(reinterpret_cast<const char *>(s.data()), s.size());
}

// Wrap a Python callable into a C++ std::function that the engine may copy and
// invoke from background threads that do not hold the GIL. Capturing the
// py::function directly would inc_ref without the GIL (a crash); a shared_ptr
// keeps copies GIL-free, and the callable is invoked / destroyed under the GIL.
// shared_ptr deleter that reacquires the GIL: the engine destroys its copies of
// these std::functions from background threads / under gil_scoped_release (e.g.
// in stop()), and destroying a py::function without the GIL aborts.
inline std::shared_ptr<py::function> make_gil_safe_holder(const py::function &fn) {
  return std::shared_ptr<py::function>(new py::function(fn), [](py::function *p) {
    py::gil_scoped_acquire gil;
    delete p;
  });
}

// Same GIL-safe holder pattern for an arbitrary py::object (e.g. a
// concurrent.futures.Future captured into a background reply callback).
// A GIL-safe wrapper for the abandon callback of call_async(): runs the Python
// callable with the GIL held, swallowing (reporting) a Python exception the
// way the reply wrappers do.
inline std::function<void()> wrap_abandon_callback(const py::function &fn, const char *where) {
  auto cb = make_gil_safe_holder(fn);
  return [cb, where]() {
    py::gil_scoped_acquire gil;
    try {
      (*cb)();
    } catch (py::error_already_set &e) {
      e.discard_as_unraisable(where);
    }
  };
}

inline std::shared_ptr<py::object> make_gil_safe_object(const py::object &obj) {
  return std::shared_ptr<py::object>(new py::object(obj), [](py::object *p) {
    py::gil_scoped_acquire gil;
    delete p;
  });
}

Rtps::sample_callback_t wrap_sample_callback(const py::function &fn) {
  if (!fn) {
    return {};
  }
  auto cb = make_gil_safe_holder(fn);
  return [cb](std::span<const uint8_t> payload) {
    py::gil_scoped_acquire gil;
    try {
      (*cb)(to_bytes(payload));
    } catch (py::error_already_set &e) {
      e.discard_as_unraisable("RtpsParticipant on_sample");
    }
  };
}

Rtps::matched_callback_t wrap_matched_callback(const py::function &fn) {
  if (!fn) {
    return {};
  }
  auto cb = make_gil_safe_holder(fn);
  return [cb]() {
    py::gil_scoped_acquire gil;
    try {
      (*cb)();
    } catch (py::error_already_set &e) {
      e.discard_as_unraisable("RtpsParticipant matched callback");
    }
  };
}

std::vector<uint8_t> to_vec(const py::bytes &b) {
  std::string s = b;
  return std::vector<uint8_t>(s.begin(), s.end());
}

// A service handler: Python callable(bytes) -> bytes. Runs on an engine thread.
Rtps::service_handler_t wrap_service_handler(const py::function &fn) {
  auto cb = make_gil_safe_holder(fn);
  return [cb](std::span<const uint8_t> request) -> std::vector<uint8_t> {
    py::gil_scoped_acquire gil;
    try {
      py::object r = (*cb)(to_bytes(request));
      if (r.is_none()) {
        return {};
      }
      return to_vec(r.cast<py::bytes>());
    } catch (py::error_already_set &e) {
      e.discard_as_unraisable("RtpsParticipant service handler");
      return {};
    }
  };
}

// A Python timeout in seconds as milliseconds: anything that is not positive
// clamps to 0, and anything beyond what a 64-bit millisecond count can hold
// (about 2.9e8 years) clamps to milliseconds::max() instead of overflowing.
static std::chrono::milliseconds seconds_to_ms(double seconds) {
  using ms = std::chrono::milliseconds;
  if (!(seconds > 0.0)) {
    return ms{0};
  }
  constexpr double max_seconds = static_cast<double>(ms::max().count()) / 1000.0;
  if (seconds >= max_seconds) {
    return ms::max();
  }
  return std::chrono::duration_cast<ms>(std::chrono::duration<double>(seconds));
}

// A reply callback for call_async: Python callable(bytes).
Rtps::ServiceClient::reply_callback_t wrap_reply_callback(const py::function &fn) {
  auto cb = make_gil_safe_holder(fn);
  return [cb](std::span<const uint8_t> reply) {
    py::gil_scoped_acquire gil;
    try {
      (*cb)(to_bytes(reply));
    } catch (py::error_already_set &e) {
      e.discard_as_unraisable("RtpsParticipant reply callback");
    }
  };
}

// Python-facing Config: like Rtps::Config but with py::function callbacks.
struct PyRtpsConfig {
  std::string interface_address{};
  std::string interface_netmask{};
  py::function on_publisher_matched{};
  py::function on_subscriber_matched{};
  espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  espp::QosBand metatraffic_band{espp::QosBand::High};
  espp::QosBand user_traffic_band{espp::QosBand::Normal};
  bool enable_dedicated_endpoint_ports{true};
  uint8_t max_prioritized_endpoint_ports{4};
  int service_reply_match_timeout_ms{1000};
  // The participant's tasks (Rtps::Config::Tasks): an unset entry keeps the
  // compile-time default, as in C++.
  std::size_t receive_workers{0};
  std::optional<espp::Task::BaseConfig> receive_worker_task{};
  std::optional<espp::Task::BaseConfig> reactor_task{};
  std::optional<espp::Task::BaseConfig> job_retry_task{};
  std::optional<espp::Task::BaseConfig> protocol_task{};
  std::optional<espp::Task::BaseConfig> reply_hold_task{};
};

Rtps::Config to_config(const PyRtpsConfig &pc) {
  return Rtps::Config{
      .interface_address = pc.interface_address,
      .interface_netmask = pc.interface_netmask,
      .on_publisher_matched = wrap_matched_callback(pc.on_publisher_matched),
      .on_subscriber_matched = wrap_matched_callback(pc.on_subscriber_matched),
      .log_level = pc.log_level,
      .metatraffic_band = pc.metatraffic_band,
      .user_traffic_band = pc.user_traffic_band,
      .enable_dedicated_endpoint_ports = pc.enable_dedicated_endpoint_ports,
      .max_prioritized_endpoint_ports = pc.max_prioritized_endpoint_ports,
      .service_reply_match_timeout = std::chrono::milliseconds(pc.service_reply_match_timeout_ms),
      .tasks =
          {
              .receive_workers = pc.receive_workers,
              .receive_worker = pc.receive_worker_task,
              .reactor = pc.reactor_task,
              .job_retry = pc.job_retry_task,
              .protocol = pc.protocol_task,
              .reply_hold = pc.reply_hold_task,
          },
  };
}

// The action server's execute-worker settings (Rtps::ActionConfig), shared by
// the ROS 2 and the native server bindings.
Rtps::ActionConfig action_config(const std::string &action, const std::string &type_name,
                                 espp::QosBand band, std::optional<espp::Dscp> dscp,
                                 std::size_t max_goals_in_flight, std::size_t execute_workers,
                                 const std::optional<espp::Task::BaseConfig> &execute_task) {
  Rtps::ActionConfig config{.action = action,
                            .type_name = type_name,
                            .band = band,
                            .dscp = dscp,
                            .max_goals_in_flight = max_goals_in_flight,
                            .execute_workers = execute_workers};
  if (execute_task) {
    config.execute_task = *execute_task;
  }
  return config;
}

py::function as_function(const py::object &obj) {
  if (obj.is_none()) {
    return py::function{};
  }
  return obj.cast<py::function>();
}

} // namespace

void py_init_rtps(py::module &m) {
  auto rtps = py::class_<Rtps>(
      m, "RtpsParticipant",
      "RTPS/DDS participant (embeddedRTPS engine) for pub/sub interop with FastDDS and ROS 2.\n"
      "Payloads are CDR-encapsulated bytes (see the cdr component / struct.pack).\n"
      "For ROS 2 use topic 'rt/<name>' and type '<pkg>::msg::dds_::<Type>_'.");

  py::enum_<Rtps::Reliability>(rtps, "Reliability")
      .value("BEST_EFFORT", Rtps::Reliability::BEST_EFFORT)
      .value("RELIABLE", Rtps::Reliability::RELIABLE);

  py::class_<PyRtpsConfig>(rtps, "Config")
      .def(py::init([](std::string interface_address, const py::object &on_publisher_matched,
                       const py::object &on_subscriber_matched, espp::Logger::Verbosity log_level,
                       espp::QosBand metatraffic_band, espp::QosBand user_traffic_band,
                       bool enable_dedicated_endpoint_ports, uint8_t max_prioritized_endpoint_ports,
                       std::string interface_netmask, int service_reply_match_timeout_ms,
                       std::size_t receive_workers,
                       std::optional<espp::Task::BaseConfig> receive_worker_task,
                       std::optional<espp::Task::BaseConfig> reactor_task,
                       std::optional<espp::Task::BaseConfig> job_retry_task,
                       std::optional<espp::Task::BaseConfig> protocol_task,
                       std::optional<espp::Task::BaseConfig> reply_hold_task) {
             PyRtpsConfig c;
             c.interface_address = std::move(interface_address);
             c.interface_netmask = std::move(interface_netmask);
             c.on_publisher_matched = as_function(on_publisher_matched);
             c.on_subscriber_matched = as_function(on_subscriber_matched);
             c.log_level = log_level;
             c.metatraffic_band = metatraffic_band;
             c.user_traffic_band = user_traffic_band;
             c.enable_dedicated_endpoint_ports = enable_dedicated_endpoint_ports;
             c.max_prioritized_endpoint_ports = max_prioritized_endpoint_ports;
             c.service_reply_match_timeout_ms = service_reply_match_timeout_ms;
             c.receive_workers = receive_workers;
             c.receive_worker_task = std::move(receive_worker_task);
             c.reactor_task = std::move(reactor_task);
             c.job_retry_task = std::move(job_retry_task);
             c.protocol_task = std::move(protocol_task);
             c.reply_hold_task = std::move(reply_hold_task);
             return c;
           }),
           py::arg("interface_address") = std::string{},
           py::arg("on_publisher_matched") = py::none(),
           py::arg("on_subscriber_matched") = py::none(),
           py::arg("log_level") = espp::Logger::Verbosity::WARN,
           py::arg("metatraffic_band") = espp::QosBand::High,
           py::arg("user_traffic_band") = espp::QosBand::Normal,
           py::arg("enable_dedicated_endpoint_ports") = true,
           py::arg("max_prioritized_endpoint_ports") = 4,
           py::arg("interface_netmask") = std::string{},
           py::arg("service_reply_match_timeout_ms") = 1000, py::arg("receive_workers") = 0,
           py::arg("receive_worker_task") = py::none(), py::arg("reactor_task") = py::none(),
           py::arg("job_retry_task") = py::none(), py::arg("protocol_task") = py::none(),
           py::arg("reply_hold_task") = py::none(),
           "interface_address: IPv4 address of the interface to use (empty: auto-detect).\n"
           "interface_netmask: its netmask (\"255.255.128.0\") or prefix length (\"17\"), "
           "which decides which of a multi-homed peer's locators is on our network; empty: "
           "the netmask of the interface that owns interface_address.\n"
           "service_reply_match_timeout_ms: how long a service reply is held for the "
           "requester's reply reader to be matched before it is written anyway (0: never held).\n"
           "receive_workers: the transport's receive worker tasks (0: the default, 2).\n"
           "receive_worker_task / reactor_task / job_retry_task / protocol_task / "
           "reply_hold_task: an espp.Task.BaseConfig (stack size, priority, core id) for "
           "that task; None keeps the compile-time default (menuconfig RTPS -> Tasks).")
      .def_readwrite("interface_address", &PyRtpsConfig::interface_address)
      .def_readwrite("interface_netmask", &PyRtpsConfig::interface_netmask)
      .def_readwrite("on_publisher_matched", &PyRtpsConfig::on_publisher_matched)
      .def_readwrite("on_subscriber_matched", &PyRtpsConfig::on_subscriber_matched)
      .def_readwrite("log_level", &PyRtpsConfig::log_level)
      .def_readwrite("metatraffic_band", &PyRtpsConfig::metatraffic_band)
      .def_readwrite("user_traffic_band", &PyRtpsConfig::user_traffic_band)
      .def_readwrite("enable_dedicated_endpoint_ports",
                     &PyRtpsConfig::enable_dedicated_endpoint_ports)
      .def_readwrite("max_prioritized_endpoint_ports",
                     &PyRtpsConfig::max_prioritized_endpoint_ports)
      .def_readwrite("service_reply_match_timeout_ms",
                     &PyRtpsConfig::service_reply_match_timeout_ms)
      .def_readwrite("receive_workers", &PyRtpsConfig::receive_workers)
      .def_readwrite("receive_worker_task", &PyRtpsConfig::receive_worker_task)
      .def_readwrite("reactor_task", &PyRtpsConfig::reactor_task)
      .def_readwrite("job_retry_task", &PyRtpsConfig::job_retry_task)
      .def_readwrite("protocol_task", &PyRtpsConfig::protocol_task)
      .def_readwrite("reply_hold_task", &PyRtpsConfig::reply_hold_task);

  rtps.def(py::init([](const PyRtpsConfig &config) { return new Rtps(to_config(config)); }),
           py::arg("config") = PyRtpsConfig{})
      .def("start", &Rtps::start, py::call_guard<py::gil_scoped_release>(),
           "Start the participant (transport + SPDP/SEDP discovery).")
      .def("stop", &Rtps::stop, py::call_guard<py::gil_scoped_release>(),
           "Stop the participant and its discovery/transport threads.")
      .def("is_started", &Rtps::is_started)
      .def(
          "add_writer",
          [](Rtps &self, const std::string &topic, const std::string &type_name, bool reliable,
             espp::QosBand band, std::optional<espp::Dscp> dscp) {
            return self.add_writer({.topic = topic,
                                    .type_name = type_name,
                                    .reliability = reliable ? Rtps::Reliability::RELIABLE
                                                            : Rtps::Reliability::BEST_EFFORT,
                                    .band = band,
                                    .dscp = dscp});
          },
          py::arg("topic"), py::arg("type_name"), py::arg("reliable") = false,
          py::arg("band") = espp::QosBand::Normal, py::arg("dscp") = py::none(),
          py::call_guard<py::gil_scoped_release>(),
          "Add a publishing endpoint. A non-Normal band (or a dscp) requests a dedicated,\n"
          "band-scheduled (and DSCP-marked) unicast port for the writer (rationed).")
      .def(
          "add_reader",
          [](Rtps &self, const std::string &topic, const std::string &type_name, bool reliable,
             const py::object &on_sample, espp::QosBand band, std::optional<espp::Dscp> dscp) {
            // wrap under the GIL (we hold it here), then release for the engine call
            auto cb = wrap_sample_callback(as_function(on_sample));
            py::gil_scoped_release release;
            return self.add_reader({.topic = topic,
                                    .type_name = type_name,
                                    .reliability = reliable ? Rtps::Reliability::RELIABLE
                                                            : Rtps::Reliability::BEST_EFFORT,
                                    .on_sample = std::move(cb),
                                    .band = band,
                                    .dscp = dscp});
          },
          py::arg("topic"), py::arg("type_name"), py::arg("reliable") = false,
          py::arg("on_sample") = py::none(), py::arg("band") = espp::QosBand::Normal,
          py::arg("dscp") = py::none(),
          "Add a subscribing endpoint; on_sample receives each sample as bytes. A non-Normal\n"
          "band (or a dscp) requests a dedicated receive port (banded readers fall back to\n"
          "deferred banded dispatch when no dedicated port is available).")
      .def("matched_writers", &Rtps::matched_writers, py::arg("topic"),
           py::call_guard<py::gil_scoped_release>(),
           "Distinct remote writers matched to any local reader on topic right now (0 without\n"
           "such a reader; a writer matched to several readers on the topic counts once). A peer\n"
           "that deletes its publisher or leaves the domain drops out as soon as its dispose\n"
           "arrives, a silent peer when its lease runs out.")
      .def("matched_readers", &Rtps::matched_readers, py::arg("topic"),
           py::call_guard<py::gil_scoped_release>(),
           "Remote readers matched to the writer on topic right now (0 without such a writer).")
      .def(
          "publish",
          [](Rtps &self, const std::string &topic, const py::bytes &data) {
            std::string s = data;
            const std::vector<uint8_t> payload(s.begin(), s.end());
            py::gil_scoped_release release;
            return self.publish(topic, payload);
          },
          py::arg("topic"), py::arg("data"),
          "Publish a CDR-encapsulated sample (bytes) on a topic added with add_writer().");

  // ---- Services (RMI, ROS 2-interoperable) --------------------------------
  py::class_<Rtps::ServiceClient, std::shared_ptr<Rtps::ServiceClient>>(
      rtps, "ServiceClient", "Handle for calling a ROS 2-interoperable service.")
      .def("server_count", &Rtps::ServiceClient::server_count,
           "Number of servers matched right now (0 until discovery has run).")
      .def("pending_count", &Rtps::ServiceClient::pending_count,
           "Requests sent and not yet answered (expired ones, past request_timeout, are "
           "dropped first).")
      .def(
          "wait_for_server",
          [](Rtps::ServiceClient &self, double timeout) {
            py::gil_scoped_release rel;
            return self.wait_for_server(seconds_to_ms(timeout));
          },
          py::arg("timeout") = 5.0,
          "Block until a server is matched (like rclpy's wait_for_service). A call made "
          "before that is refused, since its wire form is chosen from the matched servers. "
          "Returns False on timeout.")
      .def(
          "call",
          [](Rtps::ServiceClient &self, const py::bytes &request, double timeout) -> py::object {
            auto req = to_vec(request);
            std::optional<std::vector<uint8_t>> r;
            {
              py::gil_scoped_release rel;
              r = self.call(req, std::chrono::milliseconds(static_cast<long>(timeout * 1000)));
            }
            return r ? py::object(to_bytes(*r)) : py::none();
          },
          py::arg("request"), py::arg("timeout") = 5.0,
          "Blocking call (RMI). Returns the reply bytes, or None on timeout.")
      .def(
          "call_async",
          [](Rtps::ServiceClient &self, const py::bytes &request, const py::function &on_reply,
             const py::object &on_abandon) {
            auto cb = wrap_reply_callback(on_reply);
            auto req = to_vec(request);
            if (on_abandon.is_none()) {
              py::gil_scoped_release rel;
              return self.call_async(req, std::move(cb));
            }
            auto ab = wrap_abandon_callback(as_function(on_abandon), "ServiceClient.call_async");
            py::gil_scoped_release rel;
            return self.call_async(req, std::move(cb), std::move(ab));
          },
          py::arg("request"), py::arg("on_reply"), py::arg("on_abandon") = py::none(),
          "Async call (AMI): on_reply(bytes) is invoked when the reply arrives; on_abandon()\n"
          "(optional) exactly once when the request is given up on instead - pruned past\n"
          "request_timeout, or abandoned by stop() - and never after on_reply.")
      .def(
          "call_future",
          [](Rtps::ServiceClient &self, const py::bytes &request) {
            // Completed exactly once: by the reply, or by the abandonment (the
            // C++ client calls back for one or the other, never both), or here
            // when the request could not be queued.
            py::object fut = py::module_::import("concurrent.futures").attr("Future")();
            auto fut_holder = make_gil_safe_object(fut);
            auto req = to_vec(request);
            bool queued;
            {
              py::gil_scoped_release rel;
              queued = self.call_async(
                  req,
                  [fut_holder](std::span<const uint8_t> reply) {
                    py::gil_scoped_acquire gil;
                    try {
                      (*fut_holder).attr("set_result")(to_bytes(reply));
                    } catch (py::error_already_set &e) {
                      e.discard_as_unraisable("ServiceClient.call_future");
                    }
                  },
                  [fut_holder]() {
                    py::gil_scoped_acquire gil;
                    try {
                      (*fut_holder).attr("set_result")(py::none());
                    } catch (py::error_already_set &e) {
                      e.discard_as_unraisable("ServiceClient.call_future (abandon)");
                    }
                  });
            }
            if (!queued) {
              // The abandon callback may have completed it first (a request
              // given up on before its publication was refused): a second
              // set_result raises InvalidStateError, which is the one benign
              // outcome here.
              try {
                fut.attr("set_result")(py::none());
              } catch (py::error_already_set &e) {
                e.discard_as_unraisable("call_future (refused after abandon)");
              }
            }
            return fut;
          },
          py::arg("request"),
          "Async call (AMI): returns a concurrent.futures.Future for the reply bytes. The\n"
          "result is None if the request could not be queued, or was given up on without a\n"
          "reply (request_timeout, stop()). Use fut.result(timeout=...).");

  rtps.def(
          "add_service_server",
          [](Rtps &self, const std::string &service, const std::string &type_name,
             const py::function &handler, espp::QosBand band, std::optional<espp::Dscp> dscp) {
            auto h = wrap_service_handler(handler);
            py::gil_scoped_release rel;
            return self.add_service_server({service, type_name, band, dscp}, std::move(h));
          },
          py::arg("service"), py::arg("type_name"), py::arg("handler"),
          py::arg("band") = espp::QosBand::Normal, py::arg("dscp") = py::none(),
          "Add a ROS 2 service server; handler(request_bytes) -> reply_bytes.")
      .def(
          "add_service_client",
          [](Rtps &self, const std::string &service, const std::string &type_name,
             espp::QosBand band, std::optional<espp::Dscp> dscp, double request_timeout,
             std::size_t max_pending_requests) {
            py::gil_scoped_release rel;
            return self.add_service_client({.service = service,
                                            .type_name = type_name,
                                            .band = band,
                                            .dscp = dscp,
                                            .request_timeout = seconds_to_ms(request_timeout),
                                            .max_pending_requests = max_pending_requests});
          },
          py::arg("service"), py::arg("type_name"), py::arg("band") = espp::QosBand::Normal,
          py::arg("dscp") = py::none(), py::arg("request_timeout") = 30.0,
          py::arg("max_pending_requests") = 64,
          "Add a ROS 2 service client. A request unanswered for request_timeout seconds is\n"
          "abandoned (0 = never); at most max_pending_requests wait at once (0 = unlimited).");

  // ---- Actions (AMI, ROS 2-interoperable) ---------------------------------
  py::class_<Rtps::ActionGoalHandle>(
      rtps, "ActionGoalHandle", "Server-side handle to a running goal (in the execute callback).")
      .def("goal", [](Rtps::ActionGoalHandle &h) { return to_bytes(h.goal()); })
      .def(
          "publish_feedback",
          [](Rtps::ActionGoalHandle &h, const py::bytes &fb) {
            auto v = to_vec(fb);
            py::gil_scoped_release rel;
            h.publish_feedback(v);
          },
          py::arg("feedback"))
      .def(
          "succeed",
          [](Rtps::ActionGoalHandle &h, const py::bytes &result) {
            auto v = to_vec(result);
            py::gil_scoped_release rel;
            h.succeed(v);
          },
          py::arg("result"))
      .def(
          "abort",
          [](Rtps::ActionGoalHandle &h, const py::bytes &result) {
            auto v = to_vec(result);
            py::gil_scoped_release rel;
            h.abort(v);
          },
          py::arg("result"))
      .def(
          "canceled",
          [](Rtps::ActionGoalHandle &h, const py::bytes &result) {
            auto v = to_vec(result);
            py::gil_scoped_release rel;
            h.canceled(v);
          },
          py::arg("result"), "Terminate the goal CANCELED (in response to a cancel request).")
      .def("is_canceling", &Rtps::ActionGoalHandle::is_canceling);

  py::class_<Rtps::ActionClient, std::shared_ptr<Rtps::ActionClient>>(
      rtps, "ActionClient", "Handle for driving a ROS 2-interoperable action.")
      .def("server_count", &Rtps::ActionClient::server_count,
           "Number of action servers matched on all three services (0 until discovery has run).")
      .def(
          "wait_for_server",
          [](Rtps::ActionClient &self, double timeout) {
            py::gil_scoped_release rel;
            return self.wait_for_server(seconds_to_ms(timeout));
          },
          py::arg("timeout") = 5.0,
          "Block until an action server is matched (like rclpy_action's wait_for_server). A "
          "goal sent before that is refused. Returns False on timeout.")
      .def(
          "send_goal",
          [](Rtps::ActionClient &self, const py::bytes &goal, const py::function &on_feedback,
             const py::function &on_result) -> py::object {
            auto fb = make_gil_safe_holder(on_feedback);
            auto rc = make_gil_safe_holder(on_result);
            auto goal_v = to_vec(goal);
            std::optional<Rtps::GoalId> gid;
            {
              py::gil_scoped_release rel;
              gid = self.send_goal(
                  goal_v,
                  [fb](std::span<const uint8_t> f) {
                    py::gil_scoped_acquire gil;
                    try {
                      (*fb)(to_bytes(f));
                    } catch (py::error_already_set &e) {
                      e.discard_as_unraisable("action feedback");
                    }
                  },
                  [rc](int8_t status, std::span<const uint8_t> r) {
                    py::gil_scoped_acquire gil;
                    try {
                      (*rc)(status, to_bytes(r));
                    } catch (py::error_already_set &e) {
                      e.discard_as_unraisable("action result");
                    }
                  });
            }
            if (!gid) {
              return py::none();
            }
            return py::object(py::bytes(reinterpret_cast<const char *>(gid->data()), gid->size()));
          },
          py::arg("goal"), py::arg("on_feedback"), py::arg("on_result"),
          "Send a goal. on_feedback(bytes); on_result(status:int, bytes). Returns the goal id.")
      .def(
          "cancel_goal",
          [](Rtps::ActionClient &self, const py::bytes &goal_id, const py::object &on_response) {
            auto id_v = to_vec(goal_id);
            Rtps::GoalId id{};
            if (id_v.size() != id.size()) {
              return false;
            }
            std::copy(id_v.begin(), id_v.end(), id.begin());
            Rtps::ActionClient::cancel_callback_t cb = nullptr;
            if (!on_response.is_none()) {
              auto holder = make_gil_safe_holder(as_function(on_response));
              cb = [holder](int8_t return_code, std::size_t goals_canceling) {
                py::gil_scoped_acquire gil;
                try {
                  (*holder)(static_cast<int>(return_code), goals_canceling);
                } catch (py::error_already_set &e) {
                  e.discard_as_unraisable("action cancel on_response");
                }
              };
            }
            py::gil_scoped_release rel;
            return self.cancel_goal(id, std::move(cb));
          },
          py::arg("goal_id"), py::arg("on_response") = py::none(),
          "Request cancellation of a goal by its id (from send_goal). on_response(return_code,\n"
          "goals_canceling) gets the server's verdict (action_msgs/srv/CancelGoal: 0 NONE,\n"
          "1 REJECTED, 2 UNKNOWN_GOAL_ID, 3 GOAL_TERMINATED).");

  rtps.def(
      "add_action_server",
      [](Rtps &self, const std::string &action, const std::string &type_name,
         const py::function &on_goal, const py::function &execute, espp::QosBand band,
         std::optional<espp::Dscp> dscp, std::size_t max_goals_in_flight,
         std::size_t execute_workers, const std::optional<espp::Task::BaseConfig> &execute_task) {
        auto og = make_gil_safe_holder(on_goal);
        auto ex = make_gil_safe_holder(execute);
        py::gil_scoped_release rel;
        return self.add_action_server(
            action_config(action, type_name, band, dscp, max_goals_in_flight, execute_workers,
                          execute_task),
            [og](const Rtps::GoalId &, std::span<const uint8_t> goal) -> bool {
              py::gil_scoped_acquire gil;
              try {
                return (*og)(to_bytes(goal)).cast<bool>();
              } catch (py::error_already_set &e) {
                e.discard_as_unraisable("action on_goal");
                return false;
              }
            },
            [ex](Rtps::ActionGoalHandle h) {
              py::gil_scoped_acquire gil;
              try {
                (*ex)(h);
              } catch (py::error_already_set &e) {
                e.discard_as_unraisable("action execute");
              }
            });
      },
      py::arg("action"), py::arg("type_name"), py::arg("on_goal"), py::arg("execute"),
      py::arg("band") = espp::QosBand::Normal, py::arg("dscp") = py::none(),
      py::arg("max_goals_in_flight") = 8, py::arg("execute_workers") = 1,
      py::arg("execute_task") = py::none(),
      "Add a ROS 2 action server. on_goal(goal_bytes)->bool; execute(ActionGoalHandle), which\n"
      "runs on one of execute_workers tasks the server owns (configured by execute_task, an\n"
      "espp.Task.BaseConfig; goals beyond the workers wait in acceptance order). A goal beyond\n"
      "max_goals_in_flight (running or waiting; 0: unlimited) is rejected before on_goal.");
  rtps.def(
      "add_action_client",
      [](Rtps &self, const std::string &action, const std::string &type_name, espp::QosBand band,
         std::optional<espp::Dscp> dscp) {
        py::gil_scoped_release rel;
        return self.add_action_client({action, type_name, band, dscp});
      },
      py::arg("action"), py::arg("type_name"), py::arg("band") = espp::QosBand::Normal,
      py::arg("dscp") = py::none(), "Add a ROS 2 action client.");

  // ---- Native (espp<->espp) services + actions ----------------------------
  py::class_<Rtps::NativeServiceClient, std::shared_ptr<Rtps::NativeServiceClient>>(
      rtps, "NativeServiceClient", "Handle for a lean native (espp<->espp) service.")
      .def(
          "call",
          [](Rtps::NativeServiceClient &self, const py::bytes &request,
             double timeout) -> py::object {
            auto req = to_vec(request);
            std::optional<std::vector<uint8_t>> r;
            {
              py::gil_scoped_release rel;
              r = self.call(req, std::chrono::milliseconds(static_cast<long>(timeout * 1000)));
            }
            return r ? py::object(to_bytes(*r)) : py::none();
          },
          py::arg("request"), py::arg("timeout") = 5.0)
      .def(
          "call_async",
          [](Rtps::NativeServiceClient &self, const py::bytes &request,
             const py::function &on_reply, const py::object &on_abandon) {
            auto cb = make_gil_safe_holder(on_reply);
            auto req = to_vec(request);
            auto reply_cb = [cb](std::span<const uint8_t> reply) {
              py::gil_scoped_acquire gil;
              try {
                (*cb)(to_bytes(reply));
              } catch (py::error_already_set &e) {
                e.discard_as_unraisable("native reply");
              }
            };
            if (on_abandon.is_none()) {
              py::gil_scoped_release rel;
              return self.call_async(req, std::move(reply_cb));
            }
            auto ab =
                wrap_abandon_callback(as_function(on_abandon), "NativeServiceClient.call_async");
            py::gil_scoped_release rel;
            return self.call_async(req, std::move(reply_cb), std::move(ab));
          },
          py::arg("request"), py::arg("on_reply"), py::arg("on_abandon") = py::none(),
          "Async call: on_reply(bytes) when the reply arrives; on_abandon() (optional) exactly\n"
          "once when the request is given up on instead (request_timeout, stop()).")
      .def(
          "call_future",
          [](Rtps::NativeServiceClient &self, const py::bytes &request) {
            // See ServiceClient.call_future: completed exactly once.
            py::object fut = py::module_::import("concurrent.futures").attr("Future")();
            auto fut_holder = make_gil_safe_object(fut);
            auto req = to_vec(request);
            bool queued;
            {
              py::gil_scoped_release rel;
              queued = self.call_async(
                  req,
                  [fut_holder](std::span<const uint8_t> reply) {
                    py::gil_scoped_acquire gil;
                    try {
                      (*fut_holder).attr("set_result")(to_bytes(reply));
                    } catch (py::error_already_set &e) {
                      e.discard_as_unraisable("NativeServiceClient.call_future");
                    }
                  },
                  [fut_holder]() {
                    py::gil_scoped_acquire gil;
                    try {
                      (*fut_holder).attr("set_result")(py::none());
                    } catch (py::error_already_set &e) {
                      e.discard_as_unraisable("NativeServiceClient.call_future (abandon)");
                    }
                  });
            }
            if (!queued) {
              // The abandon callback may have completed it first (a request
              // given up on before its publication was refused): a second
              // set_result raises InvalidStateError, which is the one benign
              // outcome here.
              try {
                fut.attr("set_result")(py::none());
              } catch (py::error_already_set &e) {
                e.discard_as_unraisable("call_future (refused after abandon)");
              }
            }
            return fut;
          },
          py::arg("request"),
          "Async call (AMI): returns a concurrent.futures.Future for the reply bytes.");

  py::class_<Rtps::NativeActionClient, std::shared_ptr<Rtps::NativeActionClient>>(
      rtps, "NativeActionClient", "Handle for a lean native (espp<->espp) action.")
      .def(
          "send_goal",
          [](Rtps::NativeActionClient &self, const py::bytes &goal, const py::function &on_feedback,
             const py::function &on_result, const py::object &on_accepted) {
            auto fb = make_gil_safe_holder(on_feedback);
            auto rc = make_gil_safe_holder(on_result);
            Rtps::NativeActionClient::accepted_callback_t acc = nullptr;
            if (!on_accepted.is_none()) {
              auto ac = make_gil_safe_holder(on_accepted.cast<py::function>());
              acc = [ac](uint32_t handle) {
                py::gil_scoped_acquire gil;
                try {
                  (*ac)(handle);
                } catch (py::error_already_set &e) {
                  e.discard_as_unraisable("native on_accepted");
                }
              };
            }
            auto goal_v = to_vec(goal);
            py::gil_scoped_release rel;
            return self.send_goal(
                goal_v,
                [fb](std::span<const uint8_t> f) {
                  py::gil_scoped_acquire gil;
                  try {
                    (*fb)(to_bytes(f));
                  } catch (py::error_already_set &e) {
                    e.discard_as_unraisable("native feedback");
                  }
                },
                [rc](uint8_t status, std::span<const uint8_t> r) {
                  py::gil_scoped_acquire gil;
                  try {
                    (*rc)(status, to_bytes(r));
                  } catch (py::error_already_set &e) {
                    e.discard_as_unraisable("native result");
                  }
                },
                std::move(acc));
          },
          py::arg("goal"), py::arg("on_feedback"), py::arg("on_result"),
          py::arg("on_accepted") = py::none())
      .def(
          "cancel_goal",
          [](Rtps::NativeActionClient &self, uint32_t goal_handle) {
            py::gil_scoped_release rel;
            return self.cancel_goal(goal_handle);
          },
          py::arg("goal_handle"));

  rtps.def(
          "add_native_service_server",
          [](Rtps &self, const std::string &service, const std::string &type_name,
             const py::function &handler, espp::QosBand band, std::optional<espp::Dscp> dscp) {
            auto h = wrap_service_handler(handler);
            py::gil_scoped_release rel;
            return self.add_native_service_server({service, type_name, band, dscp}, std::move(h));
          },
          py::arg("service"), py::arg("type_name"), py::arg("handler"),
          py::arg("band") = espp::QosBand::Normal, py::arg("dscp") = py::none())
      .def(
          "add_native_service_client",
          [](Rtps &self, const std::string &service, const std::string &type_name,
             espp::QosBand band, std::optional<espp::Dscp> dscp) {
            py::gil_scoped_release rel;
            return self.add_native_service_client({service, type_name, band, dscp});
          },
          py::arg("service"), py::arg("type_name"), py::arg("band") = espp::QosBand::Normal,
          py::arg("dscp") = py::none())
      .def(
          "add_native_action_server",
          [](Rtps &self, const std::string &action, const std::string &type_name,
             const py::function &on_goal, const py::function &execute, const py::object &on_cancel,
             espp::QosBand band, std::optional<espp::Dscp> dscp, std::size_t max_goals_in_flight,
             std::size_t execute_workers,
             const std::optional<espp::Task::BaseConfig> &execute_task) {
            auto og = make_gil_safe_holder(on_goal);
            auto ex = make_gil_safe_holder(execute);
            Rtps::native_cancel_callback_t oc = nullptr;
            if (!on_cancel.is_none()) {
              auto ocw = make_gil_safe_holder(on_cancel.cast<py::function>());
              oc = [ocw](uint32_t handle) -> bool {
                py::gil_scoped_acquire gil;
                try {
                  return (*ocw)(handle).cast<bool>();
                } catch (py::error_already_set &e) {
                  e.discard_as_unraisable("native on_cancel");
                  return false;
                }
              };
            }
            py::gil_scoped_release rel;
            return self.add_native_action_server(
                action_config(action, type_name, band, dscp, max_goals_in_flight, execute_workers,
                              execute_task),
                [og](std::span<const uint8_t> goal) -> bool {
                  py::gil_scoped_acquire gil;
                  try {
                    return (*og)(to_bytes(goal)).cast<bool>();
                  } catch (py::error_already_set &e) {
                    e.discard_as_unraisable("native on_goal");
                    return false;
                  }
                },
                [ex](Rtps::NativeGoalHandle h) {
                  py::gil_scoped_acquire gil;
                  try {
                    (*ex)(h);
                  } catch (py::error_already_set &e) {
                    e.discard_as_unraisable("native execute");
                  }
                },
                std::move(oc));
          },
          py::arg("action"), py::arg("type_name"), py::arg("on_goal"), py::arg("execute"),
          py::arg("on_cancel") = py::none(), py::arg("band") = espp::QosBand::Normal,
          py::arg("dscp") = py::none(), py::arg("max_goals_in_flight") = 8,
          py::arg("execute_workers") = 1, py::arg("execute_task") = py::none(),
          "Add a native action server. on_goal(goal_bytes)->bool; execute(NativeGoalHandle), "
          "which\n"
          "runs on one of execute_workers tasks the server owns (configured by execute_task, an\n"
          "espp.Task.BaseConfig); a goal beyond max_goals_in_flight (running or waiting; 0:\n"
          "unlimited) is rejected before on_goal; on_cancel(handle)->bool gates cancel requests.")
      .def(
          "add_native_action_client",
          [](Rtps &self, const std::string &action, const std::string &type_name,
             espp::QosBand band, std::optional<espp::Dscp> dscp) {
            py::gil_scoped_release rel;
            return self.add_native_action_client({action, type_name, band, dscp});
          },
          py::arg("action"), py::arg("type_name"), py::arg("band") = espp::QosBand::Normal,
          py::arg("dscp") = py::none());

  py::class_<Rtps::NativeGoalHandle>(rtps, "NativeGoalHandle",
                                     "Server-side handle to a running native goal.")
      .def("goal", [](Rtps::NativeGoalHandle &h) { return to_bytes(h.goal()); })
      .def("goal_handle", &Rtps::NativeGoalHandle::goal_handle)
      .def("is_canceling", &Rtps::NativeGoalHandle::is_canceling)
      .def(
          "publish_feedback",
          [](Rtps::NativeGoalHandle &h, const py::bytes &fb) {
            auto v = to_vec(fb);
            py::gil_scoped_release rel;
            h.publish_feedback(v);
          },
          py::arg("feedback"))
      .def(
          "succeed",
          [](Rtps::NativeGoalHandle &h, const py::bytes &result) {
            auto v = to_vec(result);
            py::gil_scoped_release rel;
            h.succeed(v);
          },
          py::arg("result"))
      .def(
          "abort",
          [](Rtps::NativeGoalHandle &h, const py::bytes &result) {
            auto v = to_vec(result);
            py::gil_scoped_release rel;
            h.abort(v);
          },
          py::arg("result"))
      .def(
          "canceled",
          [](Rtps::NativeGoalHandle &h, const py::bytes &result) {
            auto v = to_vec(result);
            py::gil_scoped_release rel;
            h.canceled(v);
          },
          py::arg("result"));
}
