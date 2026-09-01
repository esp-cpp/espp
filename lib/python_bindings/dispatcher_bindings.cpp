// Hand-written pybind11 bindings for the espp `stream_frame` codec and
// `espp::Dispatcher`. Both are header-only and dependency-free, so they bind
// cleanly on the host; they are kept out of the generated pybind_espp.cpp so
// regeneration never clobbers them.
//
// Exposes:
//   espp.stream_frame.{crc32, make_flags, flags_is_reply, flags_version,
//                      build_frame, Transaction, Frame, StreamParser} + constants
//   espp.Dispatcher

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "dispatcher.hpp"
#include "stream_frame.hpp"

namespace py = pybind11;
namespace sf = espp::stream_frame;

namespace {
std::span<const uint8_t> as_span(const std::string &s) {
  return {reinterpret_cast<const uint8_t *>(s.data()), s.size()};
}
py::bytes to_bytes(const std::vector<uint8_t> &v) {
  return py::bytes(reinterpret_cast<const char *>(v.data()), v.size());
}
} // namespace

void py_init_dispatcher(py::module &m) {
  auto sfm = m.def_submodule(
      "stream_frame",
      "Dependency-free CRC-32 frame codec (magic/flags/module/type/len/payload/crc)");

  sfm.attr("MAGIC") = sf::kMagic;
  sfm.attr("HEADER_SIZE") = sf::kHeaderSize;
  sfm.attr("CRC_SIZE") = sf::kCrcSize;
  sfm.attr("MAX_PAYLOAD_SIZE") = sf::kMaxPayloadSize;
  sfm.attr("MAX_FRAME_SIZE") = sf::kMaxFrameSize;
  sfm.attr("VERSION") = sf::kVersion;
  sfm.attr("FLAG_REPLY") = sf::kFlagReply;

  py::enum_<sf::Transaction>(sfm, "Transaction",
                             "Recommended standard `type` values (a protocol may define its own)")
      .value("Write", sf::Transaction::Write)
      .value("Read", sf::Transaction::Read)
      .value("WriteRead", sf::Transaction::WriteRead)
      .value("Custom", sf::Transaction::Custom);

  sfm.def(
      "crc32", [](const std::string &data, uint32_t crc) { return sf::crc32(as_span(data), crc); },
      py::arg("data"), py::arg("crc") = 0, "Standard zlib CRC-32 (chainable via the `crc` seed).");
  sfm.def("make_flags", &sf::make_flags, py::arg("reply"), py::arg("version") = sf::kVersion,
          "Compose a flags byte from a reply bit and version.");
  sfm.def("flags_is_reply", &sf::flags_is_reply, py::arg("flags"));
  sfm.def("flags_version", &sf::flags_version, py::arg("flags"));

  py::class_<sf::Frame>(sfm, "Frame", "A decoded, CRC-verified frame.")
      .def(py::init<>())
      .def_readwrite("flags", &sf::Frame::flags)
      .def_readwrite("module", &sf::Frame::module)
      .def_readwrite("type", &sf::Frame::type)
      .def_property(
          "payload", [](const sf::Frame &f) { return to_bytes(f.payload); },
          [](sf::Frame &f, const std::string &b) { f.payload.assign(b.begin(), b.end()); })
      .def("is_reply", &sf::Frame::is_reply)
      .def("version", &sf::Frame::version)
      .def("transaction", &sf::Frame::transaction)
      .def("__repr__", [](const sf::Frame &f) {
        return "<stream_frame.Frame module=" + std::to_string(f.module) +
               " type=" + std::to_string(f.type) + " reply=" + (f.is_reply() ? "True" : "False") +
               " len=" + std::to_string(f.payload.size()) + ">";
      });

  sfm.def(
      "build_frame",
      [](uint8_t module, uint8_t type, const std::string &payload, bool reply) {
        return to_bytes(sf::build_frame(reply, module, type, as_span(payload)));
      },
      py::arg("module"), py::arg("type"), py::arg("payload") = std::string(),
      py::arg("reply") = false,
      "Encode a frame. Returns empty bytes if the payload exceeds MAX_PAYLOAD_SIZE.");

  py::class_<sf::StreamParser>(sfm, "StreamParser",
                               "Incremental, resynchronizing frame parser (yields every "
                               "CRC-verified frame; it does not filter).")
      .def(py::init<>())
      .def(
          "feed",
          [](sf::StreamParser &p, const std::string &data) { return p.feed(as_span(data)); },
          py::arg("data"), "Feed received bytes; returns the list of complete frames.")
      .def("reset", &sf::StreamParser::reset)
      .def("buffered", &sf::StreamParser::buffered)
      .def("dropped_bytes", &sf::StreamParser::dropped_bytes);

  py::class_<espp::Dispatcher>(
      m, "Dispatcher", "Routes framed messages from one byte stream to per-module handlers.")
      .def(py::init<>())
      .def(
          "register_module",
          [](espp::Dispatcher &d, uint8_t module_id, py::object cb) {
            // Accept None to unregister (a py::function argument can never be None).
            if (cb.is_none()) {
              d.unregister_module(module_id);
              return;
            }
            py::function fn = cb.cast<py::function>();
            // feed() invokes the handler synchronously on the calling (Python)
            // thread, so the GIL is held throughout; own the callable via a
            // shared_ptr with a GIL-acquiring deleter for the unregister path.
            auto held =
                std::shared_ptr<py::function>(new py::function(std::move(fn)), [](py::function *f) {
                  py::gil_scoped_acquire gil;
                  delete f;
                });
            d.register_module(module_id, [held](const sf::Frame &f) {
              py::gil_scoped_acquire gil;
              // Pass a COPY: the Frame lives only for this call (in feed()'s
              // temporary vector), so a Python handler that retains it keeps an
              // independent Python-owned object, not a wrapper over freed memory.
              (*held)(py::cast(f, py::return_value_policy::copy));
            });
          },
          py::arg("module_id"), py::arg("callback"),
          "Register a handler `callback(frame: stream_frame.Frame)` for a module id "
          "(pass None to unregister).")
      .def("unregister_module", &espp::Dispatcher::unregister_module, py::arg("module_id"))
      .def("has_module", &espp::Dispatcher::has_module, py::arg("module_id"))
      .def(
          "feed", [](espp::Dispatcher &d, const std::string &data) { d.feed(as_span(data)); },
          py::arg("data"), "Parse and route received bytes to the registered module handlers.")
      .def("dispatch", &espp::Dispatcher::dispatch, py::arg("frame"))
      .def("reset", &espp::Dispatcher::reset)
      .def("buffered", &espp::Dispatcher::buffered)
      .def("dropped_bytes", &espp::Dispatcher::dropped_bytes)
      .def_static("module_of", &espp::Dispatcher::module_of, py::arg("frame"),
                  "The module id a frame routes to (its `module` byte).");
}
