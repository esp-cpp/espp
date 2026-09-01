#include <pybind11/pybind11.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

void py_init_module_espp(py::module &m);
// Hand-written bindings for the `rtps` component (see rtps_bindings.cpp for why they are not
// generated). Must run after py_init_module_espp so shared types (e.g. Logger::Verbosity) and the
// module's classes are already registered. The cdr component has no python bindings: its C++ API
// is template-based, and the python side of a message is a plain pycdr2 dataclass instead (see
// the cdr component README).
void py_init_rtps(py::module &m);
// Hand-written bindings for espp::SocketReactor (litgen cannot parse it; see
// socket_reactor_bindings.cpp). Runs after py_init_module_espp so UdpSocket / Socket::Info /
// Logger::Verbosity are already registered.
void py_init_socket_reactor(py::module &m);
// Hand-written bindings for espp::OdriveNative + the fibre stream framing helpers (std::function
// accessors with std::error_code& and std::span wire APIs; see odrive_native_bindings.cpp). Runs
// after py_init_module_espp so Logger::Verbosity is already registered.
void py_init_odrive_native(py::module &m);
// Hand-written bindings for the espp stream_frame codec (Frame / StreamParser /
// build_frame / ...) and espp::Dispatcher. Both are header-only and
// dependency-free; kept out of the generated bindings (see dispatcher_bindings.cpp).
void py_init_dispatcher(py::module &m);

// This builds the native python extension module `espp._espp`, which the
// `espp` python package (python_bindings/espp/__init__.py) re-exports.
PYBIND11_MODULE(_espp, m) {
#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif

  py_init_module_espp(m);
  py_init_rtps(m);
  py_init_socket_reactor(m);
  py_init_odrive_native(m);
  py_init_dispatcher(m);
}
