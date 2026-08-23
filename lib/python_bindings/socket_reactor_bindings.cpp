// Hand-written pybind11 bindings for espp::SocketReactor.
//
// Why hand-written (see the note in autogenerate_bindings.py): litgen/srcmlcpp cannot parse
// SocketReactor (brace-init default members with parenthesized/cast expressions trip srcmlcpp's
// brace-init fixer), and its TCP callbacks take std::unique_ptr<TcpSocket> / TcpSocket& which
// litgen cannot bind. This shim exposes a clean, GIL-correct subset for Python: the reactor
// lifecycle and the UDP-receiver path (the TCP listener/stream paths remain C++-only for now).
//
// It is kept out of the generated pybind_espp.cpp so regeneration never clobbers it.

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "dscp.hpp"
#include "qos_band.hpp"
#include "socket_reactor.hpp"
#include "udp_socket.hpp"

namespace py = pybind11;
using espp::SocketReactor;

namespace {

// Adapt a Python callable `cb(data: bytes, sender: Socket.Info) -> Optional[bytes]` into the C++
// receive callback. Like the rtps shim: capture the py::function in a shared_ptr so the reactor may
// copy the std::function off the GIL (only the shared_ptr refcount moves, which is GIL-free); the
// callable is invoked and finally destroyed under the GIL.
espp::Socket::receive_callback_fn wrap_receive_callback(const py::function &fn) {
  if (!fn) {
    return {};
  }
  // Own the py::function via a shared_ptr with a GIL-acquiring deleter: the last
  // reference may be released on a non-Python thread (a reactor pool worker
  // erasing an entry after remove()), and ~py::function must run under the GIL.
  auto cb = std::shared_ptr<py::function>(new py::function(fn), [](py::function *f) {
    py::gil_scoped_acquire gil;
    delete f;
  });
  return [cb](std::vector<uint8_t> &data,
              const espp::Socket::Info &sender) -> std::optional<std::vector<uint8_t>> {
    py::gil_scoped_acquire gil;
    // The handler runs on a reactor pool worker; a Python exception (or a bad
    // return type) must not propagate out - it would crash/stall the worker.
    // Report it and degrade to "no response".
    try {
      py::object result =
          (*cb)(py::bytes(reinterpret_cast<const char *>(data.data()), data.size()), sender);
      if (result.is_none()) {
        return std::nullopt;
      }
      // Accept either bytes or str as the response payload.
      std::string s = py::cast<std::string>(result);
      return std::vector<uint8_t>(s.begin(), s.end());
    } catch (py::error_already_set &e) {
      // Reports the traceback via sys.unraisablehook and clears the error.
      e.discard_as_unraisable("espp.SocketReactor receive callback");
      return std::nullopt;
    } catch (const std::exception &e) {
      py::print("espp.SocketReactor receive callback error:", e.what());
      return std::nullopt;
    }
  };
}

} // namespace

void py_init_socket_reactor(py::module &m) {
  py::class_<SocketReactor>(
      m, "SocketReactor", py::dynamic_attr(),
      "A select()-based event loop that multiplexes many receiver sockets onto a thread pool, "
      "instead of one thread per socket.")
      // Construct with an owned thread pool of `worker_count` workers. (Sharing an external
      // pool is available in C++ but not exposed here.)
      .def(py::init(
               [](std::size_t worker_count, bool auto_start, espp::Logger::Verbosity log_level) {
                 SocketReactor::Config config;
                 config.pool_config.worker_count = worker_count;
                 config.auto_start = auto_start;
                 config.log_level = log_level;
                 return std::make_unique<SocketReactor>(config);
               }),
           py::arg("worker_count") = 2, py::arg("auto_start") = true,
           py::arg("log_level") = espp::Logger::Verbosity::WARN)
      // Release the GIL around start()/stop(): stop() blocks waiting for
      // in-flight handlers, and a pool worker running a Python receive callback
      // needs the GIL - holding it here would deadlock.
      .def("start", &SocketReactor::start, py::call_guard<py::gil_scoped_release>(),
           "Start the select() loop (and owned pool).")
      .def("stop", &SocketReactor::stop, py::call_guard<py::gil_scoped_release>(),
           "Stop the loop and wait for in-flight handlers to finish.")
      .def("is_running", &SocketReactor::is_running)
      .def("num_registered", &SocketReactor::num_registered)
      .def("remove", &SocketReactor::remove, py::arg("id"),
           "Unregister a socket by the id returned from add_udp_receiver().")
      .def(
          "add_udp_receiver",
          [](SocketReactor &self, espp::UdpSocket &socket, std::size_t port,
             std::size_t buffer_size, const py::function &callback, espp::QosBand band,
             std::optional<espp::Dscp> dscp) -> SocketReactor::Id {
            espp::UdpSocket::ReceiveConfig rc;
            rc.port = port;
            rc.buffer_size = buffer_size;
            rc.on_receive_callback = wrap_receive_callback(callback);
            rc.band = band;
            rc.dscp = dscp;
            return self.add_udp_receiver(socket, rc);
          },
          py::arg("socket"), py::arg("port"), py::arg("buffer_size"), py::arg("callback"),
          py::arg("band") = espp::QosBand::Normal, py::arg("dscp") = std::optional<espp::Dscp>{},
          "Bind `socket` to `port` and receive on it via the reactor. `callback(data: bytes, "
          "sender) -> Optional[bytes]`; a returned bytes is sent back to the sender. `band` "
          "selects the espp.QosBand this socket's handlers are dispatched at; `dscp` (an "
          "espp.Dscp, e.g. Dscp.EF) optionally marks transmitted replies (IP_TOS, best-effort). "
          "Returns a registration id (0 == INVALID_ID on failure).")
      .def_property_readonly_static(
          "INVALID_ID", [](py::object) { return SocketReactor::INVALID_ID; },
          "The id value returned by add_* on failure.");
}
