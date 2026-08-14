// Hand-written pybind11 bindings for espp::RtpsParticipant (the facade over the
// embeddedRTPS engine in components/rtps_embedded — see its REFACTOR_PLAN.md).
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

#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "rtps_participant.hpp"

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
  py::function on_publisher_matched{};
  py::function on_subscriber_matched{};
  espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
};

Rtps::Config to_config(const PyRtpsConfig &pc) {
  return Rtps::Config{
      .interface_address = pc.interface_address,
      .on_publisher_matched = wrap_matched_callback(pc.on_publisher_matched),
      .on_subscriber_matched = wrap_matched_callback(pc.on_subscriber_matched),
      .log_level = pc.log_level,
  };
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
                       const py::object &on_subscriber_matched, espp::Logger::Verbosity log_level) {
             PyRtpsConfig c;
             c.interface_address = std::move(interface_address);
             c.on_publisher_matched = as_function(on_publisher_matched);
             c.on_subscriber_matched = as_function(on_subscriber_matched);
             c.log_level = log_level;
             return c;
           }),
           py::arg("interface_address") = std::string{},
           py::arg("on_publisher_matched") = py::none(),
           py::arg("on_subscriber_matched") = py::none(),
           py::arg("log_level") = espp::Logger::Verbosity::WARN)
      .def_readwrite("interface_address", &PyRtpsConfig::interface_address)
      .def_readwrite("on_publisher_matched", &PyRtpsConfig::on_publisher_matched)
      .def_readwrite("on_subscriber_matched", &PyRtpsConfig::on_subscriber_matched)
      .def_readwrite("log_level", &PyRtpsConfig::log_level);

  rtps.def(py::init([](const PyRtpsConfig &config) { return new Rtps(to_config(config)); }),
           py::arg("config") = PyRtpsConfig{})
      .def("start", &Rtps::start, py::call_guard<py::gil_scoped_release>(),
           "Start the participant (transport + SPDP/SEDP discovery).")
      .def("stop", &Rtps::stop, py::call_guard<py::gil_scoped_release>(),
           "Stop the participant and its discovery/transport threads.")
      .def("is_started", &Rtps::is_started)
      .def(
          "add_writer",
          [](Rtps &self, const std::string &topic, const std::string &type_name, bool reliable) {
            return self.add_writer({.topic = topic,
                                    .type_name = type_name,
                                    .reliability = reliable ? Rtps::Reliability::RELIABLE
                                                            : Rtps::Reliability::BEST_EFFORT});
          },
          py::arg("topic"), py::arg("type_name"), py::arg("reliable") = false,
          py::call_guard<py::gil_scoped_release>(), "Add a publishing endpoint.")
      .def(
          "add_reader",
          [](Rtps &self, const std::string &topic, const std::string &type_name, bool reliable,
             const py::object &on_sample) {
            // wrap under the GIL (we hold it here), then release for the engine call
            auto cb = wrap_sample_callback(as_function(on_sample));
            py::gil_scoped_release release;
            return self.add_reader({.topic = topic,
                                    .type_name = type_name,
                                    .reliability = reliable ? Rtps::Reliability::RELIABLE
                                                            : Rtps::Reliability::BEST_EFFORT,
                                    .on_sample = std::move(cb)});
          },
          py::arg("topic"), py::arg("type_name"), py::arg("reliable") = false,
          py::arg("on_sample") = py::none(),
          "Add a subscribing endpoint; on_sample receives each sample as bytes.")
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
          [](Rtps::ServiceClient &self, const py::bytes &request, const py::function &on_reply) {
            auto cb = wrap_reply_callback(on_reply);
            auto req = to_vec(request);
            py::gil_scoped_release rel;
            return self.call_async(req, std::move(cb));
          },
          py::arg("request"), py::arg("on_reply"),
          "Async call (AMI): on_reply(bytes) is invoked when the reply arrives.")
      .def(
          "call_future",
          [](Rtps::ServiceClient &self, const py::bytes &request) {
            py::object fut = py::module_::import("concurrent.futures").attr("Future")();
            auto fut_holder = make_gil_safe_object(fut);
            auto req = to_vec(request);
            bool queued;
            {
              py::gil_scoped_release rel;
              queued = self.call_async(req, [fut_holder](std::span<const uint8_t> reply) {
                py::gil_scoped_acquire gil;
                try {
                  (*fut_holder).attr("set_result")(to_bytes(reply));
                } catch (py::error_already_set &e) {
                  e.discard_as_unraisable("ServiceClient.call_future");
                }
              });
            }
            if (!queued) {
              fut.attr("set_result")(py::none());
            }
            return fut;
          },
          py::arg("request"),
          "Async call (AMI): returns a concurrent.futures.Future for the reply bytes "
          "(result is None if the request could not be queued). Use fut.result(timeout=...).");

  rtps.def(
          "add_service_server",
          [](Rtps &self, const std::string &service, const std::string &type_name,
             const py::function &handler) {
            auto h = wrap_service_handler(handler);
            py::gil_scoped_release rel;
            return self.add_service_server({service, type_name}, std::move(h));
          },
          py::arg("service"), py::arg("type_name"), py::arg("handler"),
          "Add a ROS 2 service server; handler(request_bytes) -> reply_bytes.")
      .def(
          "add_service_client",
          [](Rtps &self, const std::string &service, const std::string &type_name) {
            py::gil_scoped_release rel;
            return self.add_service_client({service, type_name});
          },
          py::arg("service"), py::arg("type_name"), "Add a ROS 2 service client.");

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
          "Send a goal. on_feedback(bytes); on_result(status:int, bytes). Returns the goal id.");

  rtps.def(
      "add_action_server",
      [](Rtps &self, const std::string &action, const std::string &type_name,
         const py::function &on_goal, const py::function &execute) {
        auto og = make_gil_safe_holder(on_goal);
        auto ex = make_gil_safe_holder(execute);
        py::gil_scoped_release rel;
        return self.add_action_server(
            {action, type_name},
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
      "Add a ROS 2 action server. on_goal(goal_bytes)->bool; execute(ActionGoalHandle).");
  rtps.def(
      "add_action_client",
      [](Rtps &self, const std::string &action, const std::string &type_name) {
        py::gil_scoped_release rel;
        return self.add_action_client({action, type_name});
      },
      py::arg("action"), py::arg("type_name"), "Add a ROS 2 action client.");

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
             const py::function &on_reply) {
            auto cb = make_gil_safe_holder(on_reply);
            auto req = to_vec(request);
            py::gil_scoped_release rel;
            return self.call_async(req, [cb](std::span<const uint8_t> reply) {
              py::gil_scoped_acquire gil;
              try {
                (*cb)(to_bytes(reply));
              } catch (py::error_already_set &e) {
                e.discard_as_unraisable("native reply");
              }
            });
          },
          py::arg("request"), py::arg("on_reply"))
      .def(
          "call_future",
          [](Rtps::NativeServiceClient &self, const py::bytes &request) {
            py::object fut = py::module_::import("concurrent.futures").attr("Future")();
            auto fut_holder = make_gil_safe_object(fut);
            auto req = to_vec(request);
            bool queued;
            {
              py::gil_scoped_release rel;
              queued = self.call_async(req, [fut_holder](std::span<const uint8_t> reply) {
                py::gil_scoped_acquire gil;
                try {
                  (*fut_holder).attr("set_result")(to_bytes(reply));
                } catch (py::error_already_set &e) {
                  e.discard_as_unraisable("NativeServiceClient.call_future");
                }
              });
            }
            if (!queued) {
              fut.attr("set_result")(py::none());
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
             const py::function &on_result) {
            auto fb = make_gil_safe_holder(on_feedback);
            auto rc = make_gil_safe_holder(on_result);
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
                });
          },
          py::arg("goal"), py::arg("on_feedback"), py::arg("on_result"));

  rtps.def(
          "add_native_service_server",
          [](Rtps &self, const std::string &service, const std::string &type_name,
             const py::function &handler) {
            auto h = wrap_service_handler(handler);
            py::gil_scoped_release rel;
            return self.add_native_service_server({service, type_name}, std::move(h));
          },
          py::arg("service"), py::arg("type_name"), py::arg("handler"))
      .def(
          "add_native_service_client",
          [](Rtps &self, const std::string &service, const std::string &type_name) {
            py::gil_scoped_release rel;
            return self.add_native_service_client({service, type_name});
          },
          py::arg("service"), py::arg("type_name"))
      .def(
          "add_native_action_server",
          [](Rtps &self, const std::string &action, const std::string &type_name,
             const py::function &on_goal, const py::function &execute) {
            auto og = make_gil_safe_holder(on_goal);
            auto ex = make_gil_safe_holder(execute);
            py::gil_scoped_release rel;
            return self.add_native_action_server(
                {action, type_name},
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
                });
          },
          py::arg("action"), py::arg("type_name"), py::arg("on_goal"), py::arg("execute"))
      .def(
          "add_native_action_client",
          [](Rtps &self, const std::string &action, const std::string &type_name) {
            py::gil_scoped_release rel;
            return self.add_native_action_client({action, type_name});
          },
          py::arg("action"), py::arg("type_name"));

  py::class_<Rtps::NativeGoalHandle>(rtps, "NativeGoalHandle",
                                     "Server-side handle to a running native goal.")
      .def("goal", [](Rtps::NativeGoalHandle &h) { return to_bytes(h.goal()); })
      .def("goal_handle", &Rtps::NativeGoalHandle::goal_handle)
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
          py::arg("result"));
}
