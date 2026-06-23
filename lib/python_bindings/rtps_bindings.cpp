// Hand-written pybind11 bindings for the `rtps` component (RtpsParticipant).
//
// Why hand-written (like cdr): RtpsParticipant exposes std::function callbacks (some taking
// std::span<const uint8_t>, which has no pybind caster), std::span publish/on_sample APIs, and a
// large nest of helper structs. litgen/srcmlcpp cannot bind these usefully. This shim exposes a
// clean, GIL-correct Python API:
//   - publish(topic, bytes) / ReaderConfig.on_sample = callable(bytes)
//   - discovery callbacks delivering ParticipantProxy / EndpointProxy objects
//   - participant lifecycle + discovery queries
//
// It is kept out of the generated pybind_espp.cpp so regeneration never clobbers it.

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "rtps.hpp"

namespace py = pybind11;
using Rtps = espp::RtpsParticipant;

namespace {

py::bytes to_bytes(std::span<const uint8_t> s) {
  return py::bytes(reinterpret_cast<const char *>(s.data()), s.size());
}

std::vector<uint8_t> bytes_to_vec(const py::bytes &data) {
  std::string s = data;
  return std::vector<uint8_t>(s.begin(), s.end());
}

// Wrap a Python callable into a C++ std::function that is safe for the rtps component to copy and
// invoke from its background (receive / discovery) threads. The rtps component copies these
// std::functions on threads that do not hold the GIL; capturing the py::function directly would
// inc_ref the Python object without the GIL (a crash). Capturing a shared_ptr instead keeps the
// std::function copies GIL-free, and the callable is invoked / destroyed only under the GIL.
template <typename Arg>
std::function<void(Arg)> wrap_callback(const py::function &fn,
                                       std::function<py::object(Arg)> to_py) {
  if (!fn) {
    return {};
  }
  auto cb = std::make_shared<py::function>(fn);
  return [cb, to_py = std::move(to_py)](Arg arg) {
    py::gil_scoped_acquire gil;
    (*cb)(to_py(arg));
  };
}

// Reader config exposed to Python: like Rtps::ReaderConfig but `on_sample` is a Python callable
// taking `bytes` (the raw CDR sample). add_reader() adapts it into the span-based C++ callback.
struct PyReaderConfig {
  std::string topic_name{};
  std::string type_name{"std_msgs/msg/UInt32"};
  Rtps::ReliabilityKind reliability{Rtps::ReliabilityKind::BEST_EFFORT};
  std::string multicast_group{};
  uint32_t entity_index{0};
  py::function on_sample{};
};

Rtps::ReaderConfig to_reader_config(const PyReaderConfig &pc) {
  Rtps::ReaderConfig rc;
  rc.topic_name = pc.topic_name;
  rc.type_name = pc.type_name;
  rc.reliability = pc.reliability;
  rc.multicast_group = pc.multicast_group;
  rc.entity_index = pc.entity_index;
  rc.on_sample = wrap_callback<std::span<const uint8_t>>(
      pc.on_sample, [](std::span<const uint8_t> data) -> py::object { return to_bytes(data); });
  return rc;
}

// Participant config exposed to Python. The discovery callbacks are Python callables (adapted the
// same GIL-safe way as on_sample). Task configs are left at their espp defaults.
struct PyRtpsConfig {
  std::string node_name{"espp_rtps"};
  uint16_t domain_id{0};
  uint16_t participant_id{0};
  std::string bind_address{"0.0.0.0"};
  std::string advertised_address{"127.0.0.1"};
  std::string metatraffic_multicast_group{"239.255.0.1"};
  std::string user_multicast_group{"239.255.0.1"};
  bool use_multicast_for_user_data{false};
  std::chrono::milliseconds announce_period{1000};
  std::string enclave{"/"};
  py::function on_participant_discovered{};
  py::function on_endpoint_discovered{};
  espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  espp::Logger::Verbosity socket_log_level{espp::Logger::Verbosity::WARN};
};

Rtps::Config to_config(const PyRtpsConfig &pc) {
  Rtps::Config c;
  c.node_name = pc.node_name;
  c.domain_id = pc.domain_id;
  c.participant_id = pc.participant_id;
  c.bind_address = pc.bind_address;
  c.advertised_address = pc.advertised_address;
  c.metatraffic_multicast_group = pc.metatraffic_multicast_group;
  c.user_multicast_group = pc.user_multicast_group;
  c.use_multicast_for_user_data = pc.use_multicast_for_user_data;
  c.announce_period = pc.announce_period;
  c.enclave = pc.enclave;
  c.log_level = pc.log_level;
  c.socket_log_level = pc.socket_log_level;
  c.on_participant_discovered = wrap_callback<const Rtps::ParticipantProxy &>(
      pc.on_participant_discovered,
      [](const Rtps::ParticipantProxy &p) -> py::object { return py::cast(p); });
  c.on_endpoint_discovered = wrap_callback<const Rtps::EndpointProxy &>(
      pc.on_endpoint_discovered,
      [](const Rtps::EndpointProxy &e) -> py::object { return py::cast(e); });
  return c;
}

} // namespace

void py_init_rtps(py::module &m) {
  auto rtps = py::class_<Rtps>(m, "RtpsParticipant", py::dynamic_attr(),
                               "Cross-platform RTPS protocol participant (discovery + best-effort "
                               "CDR-over-RTPS user data).");

  py::enum_<Rtps::ReliabilityKind>(rtps, "ReliabilityKind")
      .value("BEST_EFFORT", Rtps::ReliabilityKind::BEST_EFFORT)
      .value("RELIABLE", Rtps::ReliabilityKind::RELIABLE);

  py::class_<Rtps::GuidPrefix>(rtps, "GuidPrefix")
      .def(py::init<>())
      .def_readonly("value", &Rtps::GuidPrefix::value)
      .def("to_string", &Rtps::GuidPrefix::to_string)
      .def("__repr__", &Rtps::GuidPrefix::to_string);

  py::class_<Rtps::EntityId>(rtps, "EntityId")
      .def(py::init<>())
      .def_readonly("value", &Rtps::EntityId::value)
      .def("to_string", &Rtps::EntityId::to_string)
      .def("__repr__", &Rtps::EntityId::to_string);

  py::class_<Rtps::Guid>(rtps, "Guid")
      .def(py::init<>())
      .def_readonly("prefix", &Rtps::Guid::prefix)
      .def_readonly("entity_id", &Rtps::Guid::entity_id)
      .def("to_string", &Rtps::Guid::to_string)
      .def("__repr__", &Rtps::Guid::to_string);

  auto locator = py::class_<Rtps::Locator>(rtps, "Locator");
  py::enum_<Rtps::Locator::Kind>(locator, "Kind")
      .value("INVALID", Rtps::Locator::Kind::INVALID)
      .value("UDP_V4", Rtps::Locator::Kind::UDP_V4);
  locator.def(py::init<>())
      .def_static("udp_v4", &Rtps::Locator::udp_v4, py::arg("ipv4_address"), py::arg("port"))
      .def_readwrite("kind", &Rtps::Locator::kind)
      .def_readwrite("port", &Rtps::Locator::port)
      .def("address_string", &Rtps::Locator::address_string);

  py::class_<Rtps::PortMapping>(rtps, "PortMapping")
      .def(py::init<>())
      .def_readwrite("metatraffic_multicast", &Rtps::PortMapping::metatraffic_multicast)
      .def_readwrite("metatraffic_unicast", &Rtps::PortMapping::metatraffic_unicast)
      .def_readwrite("user_multicast", &Rtps::PortMapping::user_multicast)
      .def_readwrite("user_unicast", &Rtps::PortMapping::user_unicast);

  py::class_<Rtps::ParticipantProxy>(rtps, "ParticipantProxy")
      .def_readonly("participant_guid", &Rtps::ParticipantProxy::participant_guid)
      .def_readonly("guid_prefix", &Rtps::ParticipantProxy::guid_prefix)
      .def_readonly("name", &Rtps::ParticipantProxy::name)
      .def_readonly("enclave", &Rtps::ParticipantProxy::enclave)
      .def_readonly("address", &Rtps::ParticipantProxy::address)
      .def_readonly("ports", &Rtps::ParticipantProxy::ports)
      .def_readonly("builtin_endpoints", &Rtps::ParticipantProxy::builtin_endpoints);

  py::class_<Rtps::EndpointProxy>(rtps, "EndpointProxy")
      .def_readonly("guid", &Rtps::EndpointProxy::guid)
      .def_readonly("participant_guid", &Rtps::EndpointProxy::participant_guid)
      .def_readonly("topic_name", &Rtps::EndpointProxy::topic_name)
      .def_readonly("type_name", &Rtps::EndpointProxy::type_name)
      .def_readonly("reliability", &Rtps::EndpointProxy::reliability)
      .def_readonly("is_reader", &Rtps::EndpointProxy::is_reader)
      .def_readonly("expects_inline_qos", &Rtps::EndpointProxy::expects_inline_qos)
      .def_readonly("unicast_locator", &Rtps::EndpointProxy::unicast_locator)
      .def_readonly("multicast_locators", &Rtps::EndpointProxy::multicast_locators);

  py::class_<Rtps::WriterConfig>(rtps, "WriterConfig")
      .def(py::init<>())
      .def_readwrite("topic_name", &Rtps::WriterConfig::topic_name)
      .def_readwrite("type_name", &Rtps::WriterConfig::type_name)
      .def_readwrite("reliability", &Rtps::WriterConfig::reliability)
      .def_readwrite("multicast_group", &Rtps::WriterConfig::multicast_group)
      .def_readwrite("entity_index", &Rtps::WriterConfig::entity_index);

  py::class_<PyReaderConfig>(rtps, "ReaderConfig")
      .def(py::init<>())
      .def_readwrite("topic_name", &PyReaderConfig::topic_name)
      .def_readwrite("type_name", &PyReaderConfig::type_name)
      .def_readwrite("reliability", &PyReaderConfig::reliability)
      .def_readwrite("multicast_group", &PyReaderConfig::multicast_group)
      .def_readwrite("entity_index", &PyReaderConfig::entity_index)
      .def_readwrite("on_sample", &PyReaderConfig::on_sample,
                     "Callable invoked with the raw CDR sample (bytes) on a matching topic.");

  // Config: host-relevant fields. The discovery callbacks are Python callables receiving bound
  // proxy objects, adapted GIL-safely (see wrap_callback / PyRtpsConfig).
  py::class_<PyRtpsConfig>(rtps, "Config")
      .def(py::init<>())
      .def_readwrite("node_name", &PyRtpsConfig::node_name)
      .def_readwrite("domain_id", &PyRtpsConfig::domain_id)
      .def_readwrite("participant_id", &PyRtpsConfig::participant_id)
      .def_readwrite("bind_address", &PyRtpsConfig::bind_address)
      .def_readwrite("advertised_address", &PyRtpsConfig::advertised_address)
      .def_readwrite("metatraffic_multicast_group", &PyRtpsConfig::metatraffic_multicast_group)
      .def_readwrite("user_multicast_group", &PyRtpsConfig::user_multicast_group)
      .def_readwrite("use_multicast_for_user_data", &PyRtpsConfig::use_multicast_for_user_data)
      .def_readwrite("announce_period", &PyRtpsConfig::announce_period)
      .def_readwrite("enclave", &PyRtpsConfig::enclave)
      .def_readwrite("on_participant_discovered", &PyRtpsConfig::on_participant_discovered)
      .def_readwrite("on_endpoint_discovered", &PyRtpsConfig::on_endpoint_discovered)
      .def_readwrite("log_level", &PyRtpsConfig::log_level)
      .def_readwrite("socket_log_level", &PyRtpsConfig::socket_log_level);

  rtps.def(py::init([](const PyRtpsConfig &pc) { return std::make_unique<Rtps>(to_config(pc)); }),
           py::arg("config"))
      .def("start", &Rtps::start, py::call_guard<py::gil_scoped_release>())
      .def("stop", &Rtps::stop, py::call_guard<py::gil_scoped_release>())
      .def("is_started", &Rtps::is_started)
      .def("add_writer", &Rtps::add_writer, py::arg("writer_config"))
      .def(
          "add_reader",
          [](Rtps &self, const PyReaderConfig &rc) {
            return self.add_reader(to_reader_config(rc));
          },
          py::arg("reader_config"))
      .def("discovered_participants", &Rtps::discovered_participants)
      .def("discovered_writers", &Rtps::discovered_writers)
      .def("discovered_readers", &Rtps::discovered_readers)
      .def("writers", &Rtps::writers)
      .def("readers",
           [](const Rtps &self) {
             // Return the host-friendly reader view (without the C++ span callback).
             std::vector<PyReaderConfig> out;
             for (const auto &r : self.readers()) {
               out.push_back({r.topic_name, r.type_name, r.reliability, r.multicast_group,
                              r.entity_index, py::function{}});
             }
             return out;
           })
      .def("ports", &Rtps::ports)
      .def("participant_guid", &Rtps::participant_guid)
      .def("writer_guid", &Rtps::writer_guid, py::arg("index"))
      .def("reader_guid", &Rtps::reader_guid, py::arg("index"))
      .def(
          "publish",
          [](Rtps &self, std::string_view topic, const py::bytes &cdr_payload) {
            auto vec = bytes_to_vec(cdr_payload);
            py::gil_scoped_release release;
            return self.publish(topic, std::span<const uint8_t>{vec.data(), vec.size()});
          },
          py::arg("topic_name"), py::arg("cdr_payload"))
      .def_static("compute_port_mapping", &Rtps::compute_port_mapping, py::arg("domain_id"),
                  py::arg("participant_id"));
}
