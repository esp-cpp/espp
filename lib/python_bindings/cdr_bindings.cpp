// Hand-written pybind11 bindings for the `cdr` component.
//
// Why hand-written instead of litgen-generated:
//  - litgen/srcmlcpp cannot parse cdr.hpp (its class parser chokes on the method bodies /
//    requires-clauses), and even when coerced it produces an unusable API:
//      * CdrReader::read<T>(T& value) is an *output* reference, which pybind cannot return to
//        Python (the decoded value is lost).
//      * CdrReader holds a NON-OWNING std::span, so a generated `CdrReader(bytes)` would dangle
//        once the Python buffer is freed.
//  - This shim exposes a clean, safe, Pythonic API: write_*(value), read_*() -> Optional[...],
//    bytes in/out, and an owning reader wrapper that copies its input buffer.
//
// It is intentionally separate from the generated pybind_espp.cpp so regeneration never clobbers
// it, and it is easy to extend with new CDR helpers.

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "cdr.hpp"

namespace py = pybind11;

namespace {

std::vector<uint8_t> bytes_to_vec(const py::bytes &data) {
  // py::bytes -> std::string -> bytes copy (owning).
  std::string s = data;
  return std::vector<uint8_t>(s.begin(), s.end());
}

py::bytes span_to_bytes(std::span<const uint8_t> s) {
  return py::bytes(reinterpret_cast<const char *>(s.data()), s.size());
}

py::bytes vec_to_bytes(const std::vector<uint8_t> &v) {
  return py::bytes(reinterpret_cast<const char *>(v.data()), v.size());
}

// Owning wrapper around CdrReader: CdrReader stores a non-owning std::span, so we keep the decoded
// buffer alive for the lifetime of the reader. Declaration order matters: `storage_` must be
// initialized before `reader_` (which references it).
class PyCdrReader {
public:
  explicit PyCdrReader(const py::bytes &data,
                       espp::CdrReader::Config config = espp::CdrReader::Config{})
      : storage_(bytes_to_vec(data))
      , reader_(std::span<const uint8_t>{storage_.data(), storage_.size()}, config) {}

  static PyCdrReader
  make_body_reader(const py::bytes &data,
                   espp::CdrEncapsulation encapsulation = espp::CdrEncapsulation::CDR_LE) {
    return PyCdrReader(data, espp::CdrReader::body_config(encapsulation));
  }

  bool valid() const { return reader_.valid(); }
  espp::CdrEncapsulation encapsulation() const { return reader_.encapsulation(); }
  bool uses_little_endian() const { return reader_.uses_little_endian(); }
  size_t remaining() const { return reader_.remaining(); }
  py::bytes payload() const { return span_to_bytes(reader_.payload()); }
  py::bytes remaining_view() const { return span_to_bytes(reader_.remaining_view()); }
  bool skip(size_t length) { return reader_.skip(length); }
  bool align(size_t alignment) { return reader_.align(alignment); }

  template <typename T> std::optional<T> read() {
    T value{};
    if (!reader_.read<T>(value)) {
      return std::nullopt;
    }
    return value;
  }

  std::optional<bool> read_bool() {
    bool value = false;
    if (!reader_.read_bool(value)) {
      return std::nullopt;
    }
    return value;
  }

  std::optional<std::string> read_string() {
    std::string value;
    if (!reader_.read_string(value)) {
      return std::nullopt;
    }
    return value;
  }

  std::optional<py::bytes> read_bytes(size_t length, size_t alignment = 1) {
    std::vector<uint8_t> bytes;
    if (!reader_.read_bytes(bytes, length, alignment)) {
      return std::nullopt;
    }
    return vec_to_bytes(bytes);
  }

  template <typename T> std::optional<std::vector<T>> read_sequence() {
    std::vector<T> values;
    if (!reader_.read_sequence<T>(values)) {
      return std::nullopt;
    }
    return values;
  }

private:
  std::vector<uint8_t> storage_;
  espp::CdrReader reader_;
};

template <typename T> void add_writer_scalar(py::class_<espp::CdrWriter> &c, const char *name) {
  c.def(
      name, [](espp::CdrWriter &w, T value) { return w.write<T>(value); }, py::arg("value"));
}

template <typename T> void add_writer_sequence(py::class_<espp::CdrWriter> &c, const char *name) {
  c.def(
      name,
      [](espp::CdrWriter &w, const std::vector<T> &values) {
        return w.write_sequence<T>(std::span<const T>{values.data(), values.size()});
      },
      py::arg("values"));
}

template <typename T> void add_reader_scalar(py::class_<PyCdrReader> &c, const char *name) {
  c.def(name, &PyCdrReader::read<T>);
}

template <typename T> void add_reader_sequence(py::class_<PyCdrReader> &c, const char *name) {
  c.def(name, &PyCdrReader::read_sequence<T>);
}

} // namespace

void py_init_cdr(py::module &m) {
  py::enum_<espp::CdrEncapsulation>(m, "CdrEncapsulation",
                                    "Supported CDR encapsulation identifiers.")
      .value("CDR_BE", espp::CdrEncapsulation::CDR_BE)
      .value("CDR_LE", espp::CdrEncapsulation::CDR_LE)
      .value("PL_CDR_BE", espp::CdrEncapsulation::PL_CDR_BE)
      .value("PL_CDR_LE", espp::CdrEncapsulation::PL_CDR_LE);

  // ---- CdrWriter (owns its buffer, so it is safe to bind directly) ----
  auto writer = py::class_<espp::CdrWriter>(m, "CdrWriter", py::dynamic_attr(),
                                            "Helper for building CDR/XCDR1-style byte streams.");

  py::class_<espp::CdrWriter::Config>(writer, "Config")
      .def(py::init<>())
      .def_readwrite("encapsulation", &espp::CdrWriter::Config::encapsulation)
      .def_readwrite("include_encapsulation", &espp::CdrWriter::Config::include_encapsulation);

  writer.def(py::init<>())
      .def(py::init<const espp::CdrWriter::Config &>(), py::arg("config"))
      .def_static("make_body_writer", &espp::CdrWriter::make_body_writer,
                  py::arg("encapsulation") = espp::CdrEncapsulation::CDR_LE)
      .def_static(
          "encapsulate",
          [](const py::bytes &payload, espp::CdrEncapsulation encapsulation) {
            auto vec = bytes_to_vec(payload);
            return vec_to_bytes(espp::CdrWriter::encapsulate(
                std::span<const uint8_t>{vec.data(), vec.size()}, encapsulation));
          },
          py::arg("payload"), py::arg("encapsulation") = espp::CdrEncapsulation::CDR_LE)
      .def("reset", &espp::CdrWriter::reset)
      .def("encapsulation", &espp::CdrWriter::encapsulation)
      .def("uses_little_endian", &espp::CdrWriter::uses_little_endian)
      .def("size", &espp::CdrWriter::size)
      .def("align", &espp::CdrWriter::align, py::arg("alignment"))
      .def("write_bool", &espp::CdrWriter::write_bool, py::arg("value"))
      .def("write_string", &espp::CdrWriter::write_string, py::arg("text"))
      .def(
          "write_bytes",
          [](espp::CdrWriter &w, const py::bytes &data, size_t alignment) {
            auto vec = bytes_to_vec(data);
            return w.write_bytes(std::span<const uint8_t>{vec.data(), vec.size()}, alignment);
          },
          py::arg("data"), py::arg("alignment") = 1)
      .def("buffer", [](const espp::CdrWriter &w) { return vec_to_bytes(w.buffer()); })
      .def("payload", [](const espp::CdrWriter &w) { return span_to_bytes(w.payload()); })
      .def("take_buffer", [](espp::CdrWriter &w) { return vec_to_bytes(w.take_buffer()); });

  add_writer_scalar<uint8_t>(writer, "write_uint8");
  add_writer_scalar<uint16_t>(writer, "write_uint16");
  add_writer_scalar<uint32_t>(writer, "write_uint32");
  add_writer_scalar<uint64_t>(writer, "write_uint64");
  add_writer_scalar<int8_t>(writer, "write_int8");
  add_writer_scalar<int16_t>(writer, "write_int16");
  add_writer_scalar<int32_t>(writer, "write_int32");
  add_writer_scalar<int64_t>(writer, "write_int64");
  add_writer_scalar<float>(writer, "write_float");
  add_writer_scalar<double>(writer, "write_double");
  add_writer_sequence<uint8_t>(writer, "write_sequence_uint8");
  add_writer_sequence<uint16_t>(writer, "write_sequence_uint16");
  add_writer_sequence<uint32_t>(writer, "write_sequence_uint32");
  add_writer_sequence<int32_t>(writer, "write_sequence_int32");
  add_writer_sequence<float>(writer, "write_sequence_float");
  add_writer_sequence<double>(writer, "write_sequence_double");

  // ---- CdrReader (owning wrapper; read_* return Optional, None on failure) ----
  auto reader = py::class_<PyCdrReader>(m, "CdrReader", py::dynamic_attr(),
                                        "Helper for parsing CDR/XCDR1-style byte streams. Copies "
                                        "the input buffer so it is safe to use independently.");

  py::class_<espp::CdrReader::Config>(reader, "Config")
      .def(py::init<>())
      .def_readwrite("expect_encapsulation", &espp::CdrReader::Config::expect_encapsulation)
      .def_readwrite("default_encapsulation", &espp::CdrReader::Config::default_encapsulation);

  reader
      .def(py::init<const py::bytes &, espp::CdrReader::Config>(), py::arg("data"),
           py::arg("config") = espp::CdrReader::Config{})
      .def_static("make_body_reader", &PyCdrReader::make_body_reader, py::arg("data"),
                  py::arg("encapsulation") = espp::CdrEncapsulation::CDR_LE)
      .def("valid", &PyCdrReader::valid)
      .def("encapsulation", &PyCdrReader::encapsulation)
      .def("uses_little_endian", &PyCdrReader::uses_little_endian)
      .def("remaining", &PyCdrReader::remaining)
      .def("payload", &PyCdrReader::payload)
      .def("remaining_view", &PyCdrReader::remaining_view)
      .def("skip", &PyCdrReader::skip, py::arg("length"))
      .def("align", &PyCdrReader::align, py::arg("alignment"))
      .def("read_bool", &PyCdrReader::read_bool)
      .def("read_string", &PyCdrReader::read_string)
      .def("read_bytes", &PyCdrReader::read_bytes, py::arg("length"), py::arg("alignment") = 1);

  add_reader_scalar<uint8_t>(reader, "read_uint8");
  add_reader_scalar<uint16_t>(reader, "read_uint16");
  add_reader_scalar<uint32_t>(reader, "read_uint32");
  add_reader_scalar<uint64_t>(reader, "read_uint64");
  add_reader_scalar<int8_t>(reader, "read_int8");
  add_reader_scalar<int16_t>(reader, "read_int16");
  add_reader_scalar<int32_t>(reader, "read_int32");
  add_reader_scalar<int64_t>(reader, "read_int64");
  add_reader_scalar<float>(reader, "read_float");
  add_reader_scalar<double>(reader, "read_double");
  add_reader_sequence<uint8_t>(reader, "read_sequence_uint8");
  add_reader_sequence<uint16_t>(reader, "read_sequence_uint16");
  add_reader_sequence<uint32_t>(reader, "read_sequence_uint32");
  add_reader_sequence<int32_t>(reader, "read_sequence_int32");
  add_reader_sequence<float>(reader, "read_sequence_float");
  add_reader_sequence<double>(reader, "read_sequence_double");
}
