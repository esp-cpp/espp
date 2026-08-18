// Hand-written pybind11 bindings for espp::OdriveNative (the transport-agnostic
// ODrive legacy native / Fibre endpoint protocol server in
// components/odrive_native) plus its stream-framing helpers.
//
// Why hand-written (like rtps/cdr): the registration API takes std::function
// getters/setters with std::error_code& out-params and the wire API uses
// std::span<const uint8_t> — neither of which litgen can bind generically. The
// shim exposes a pythonic API instead:
//   dev = espp.OdriveNative()
//   dev.register_float_property("axis0.controller.input_pos",
//                               getter=lambda: pos, setter=on_set)  # setter optional
//   resp = dev.process_bytes(request_bytes)   # bytes -> bytes (empty = no response)
//   dev.json(), dev.json_crc()
// plus module-level odrive_crc16 / odrive_crc8 / odrive_stream_frame and an
// OdriveStreamDeframer class for the UART stream framing.
//
// GIL notes: process_bytes() is called from Python (GIL held) and invokes the
// registered getters/setters synchronously on the same thread, so callbacks are
// simply invoked under the caller's GIL. The gil_scoped_acquire in the wrappers
// is a cheap no-op there and keeps the callbacks safe if a future embedder
// calls process_bytes() from a non-Python thread.
//
// It is kept out of the generated pybind_espp.cpp so regeneration never
// clobbers it.

#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "detail/odrive_native_stream.hpp" // stream framing + CRC8 (not pulled in by the public header)
#include "odrive_native.hpp"

namespace py = pybind11;

namespace {

py::bytes vec_to_bytes(const std::vector<uint8_t> &v) {
  return py::bytes(reinterpret_cast<const char *>(v.data()), v.size());
}

std::vector<uint8_t> bytes_to_vec(const py::bytes &b) {
  std::string s = b; // pybind copies the buffer
  return std::vector<uint8_t>(s.begin(), s.end());
}

// Define one register_<type>_property binding. The Python getter is a
// zero-argument callable returning the value; the optional setter is called
// with the new value and may return False (or raise) to reject the write —
// a rejected write is reported through the component's error callback (the
// logger by default) since the wire protocol has no error channel.
template <typename T, typename RegFn>
void def_register(py::class_<espp::OdriveNative> &cls, const char *name, RegFn reg) {
  cls.def(
      name,
      [reg](espp::OdriveNative &self, const std::string &path, const py::function &getter,
            const py::object &setter) {
        std::function<T()> g = nullptr;
        if (!getter.is_none()) {
          g = [getter]() -> T {
            py::gil_scoped_acquire gil;
            return getter().template cast<T>();
          };
        }
        std::function<bool(T, std::error_code &)> s = nullptr;
        if (!setter.is_none()) {
          auto sf = setter.cast<py::function>();
          s = [sf](T v, std::error_code &ec) -> bool {
            py::gil_scoped_acquire gil;
            try {
              py::object r = sf(v);
              const bool ok = r.is_none() ? true : r.cast<bool>();
              if (!ok)
                ec = std::make_error_code(std::errc::invalid_argument);
              return ok;
            } catch (const py::error_already_set &) {
              ec = std::make_error_code(std::errc::invalid_argument);
              return false;
            }
          };
        }
        (self.*reg)(path, g, s);
      },
      py::arg("path"), py::arg("getter"), py::arg("setter") = py::none());
}

} // namespace

void py_init_odrive_native(py::module &m) {
  using espp::OdriveNative;
  using espp::detail::odrive_crc16;
  using espp::detail::odrive_crc8;
  using espp::detail::stream_frame;
  using espp::detail::StreamDeframer;

  // ---- CRC + UART stream-framing helpers ---------------------------------
  m.def(
      "odrive_crc16",
      [](const py::bytes &data, uint16_t init) {
        auto v = bytes_to_vec(data);
        return odrive_crc16(std::span<const uint8_t>(v.data(), v.size()), init);
      },
      py::arg("data"), py::arg("init") = 0x1337,
      "ODrive legacy CRC-16 (poly 0x3d65). init=0x1337 for packet/stream use; "
      "pass init=1 (PROTOCOL_VERSION) to compute a json_crc.");
  m.def(
      "odrive_crc8",
      [](const py::bytes &data, uint8_t init) {
        auto v = bytes_to_vec(data);
        return odrive_crc8(std::span<const uint8_t>(v.data(), v.size()), init);
      },
      py::arg("data"), py::arg("init") = 0x42, "ODrive/fibre stream CRC-8 (poly 0x37, init 0x42).");
  m.def(
      "odrive_stream_frame",
      [](const py::bytes &packet) {
        auto v = bytes_to_vec(packet);
        return vec_to_bytes(stream_frame(std::span<const uint8_t>(v.data(), v.size())));
      },
      py::arg("packet"),
      "Wrap one packet (< 128 bytes) in the fibre UART stream framing "
      "(sync, len, crc8, packet, crc16-BE). Returns b'' if the packet is too large.");

  py::class_<StreamDeframer>(m, "OdriveStreamDeframer",
                             "Stateful deframer for the fibre UART stream framing: feed received "
                             "chunks to push() and get back complete, CRC-verified packets.")
      .def(py::init<>())
      .def(
          "push",
          [](StreamDeframer &self, const py::bytes &data) {
            auto v = bytes_to_vec(data);
            auto packets = self.push(std::span<const uint8_t>(v.data(), v.size()));
            std::vector<py::bytes> out;
            out.reserve(packets.size());
            for (const auto &p : packets)
              out.push_back(vec_to_bytes(p));
            return out;
          },
          py::arg("data"), "Append received stream bytes; returns the list of decoded packets.")
      .def("buffered", &StreamDeframer::buffered,
           "Bytes currently buffered awaiting a complete frame.");

  // ---- The protocol server ------------------------------------------------
  auto cls =
      py::class_<OdriveNative>(m, "OdriveNative",
                               "Transport-agnostic server for the ODrive legacy native (Fibre "
                               "endpoint) binary protocol. Register typed properties from dotted "
                               "paths, then feed request packets to process_bytes() and send back "
                               "whatever it returns (empty = no response expected).")
          .def(py::init([](espp::Logger::Verbosity log_level) {
                 return new OdriveNative(OdriveNative::Config{.log_level = log_level});
               }),
               py::arg("log_level") = espp::Logger::Verbosity::WARN)
          .def(
              "process_bytes",
              [](OdriveNative &self, const py::bytes &data) {
                auto v = bytes_to_vec(data);
                return vec_to_bytes(
                    self.process_bytes(std::span<const uint8_t>(v.data(), v.size())));
              },
              py::arg("data"),
              "Process exactly one inbound request packet; returns the response packet "
              "bytes (empty when no response is expected / the packet is ignored).")
          .def("finalize", &OdriveNative::finalize,
               "Build (or rebuild) the JSON descriptor + CRC now (otherwise lazy).")
          .def(
              "json", [](OdriveNative &self) { return self.json(); },
              "The compact JSON endpoint descriptor (endpoint 0 blob).")
          .def(
              "json_crc", [](OdriveNative &self) { return self.json_crc(); },
              "CRC-16 of the JSON descriptor (the canary for non-zero endpoints).");

  def_register<float>(cls, "register_float_property", &OdriveNative::register_float_property);
  def_register<int8_t>(cls, "register_int8_property", &OdriveNative::register_int8_property);
  def_register<uint8_t>(cls, "register_uint8_property", &OdriveNative::register_uint8_property);
  def_register<int16_t>(cls, "register_int16_property", &OdriveNative::register_int16_property);
  def_register<uint16_t>(cls, "register_uint16_property", &OdriveNative::register_uint16_property);
  def_register<int32_t>(cls, "register_int32_property", &OdriveNative::register_int32_property);
  def_register<uint32_t>(cls, "register_uint32_property", &OdriveNative::register_uint32_property);
  def_register<int64_t>(cls, "register_int64_property", &OdriveNative::register_int64_property);
  def_register<uint64_t>(cls, "register_uint64_property", &OdriveNative::register_uint64_property);
  def_register<bool>(cls, "register_bool_property", &OdriveNative::register_bool_property);
}
