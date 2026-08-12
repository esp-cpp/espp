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
}
