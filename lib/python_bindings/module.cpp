#include <pybind11/pybind11.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

void py_init_module_espp(py::module &m);
// Hand-written bindings for the `cdr` and `rtps` components (see *_bindings.cpp for why they are
// not generated). Both must run after py_init_module_espp so shared types (e.g. Logger::Verbosity)
// and the module's classes are already registered.
void py_init_cdr(py::module &m);
void py_init_rtps(py::module &m);

// This builds the native python module `espp`
// it will be wrapped in a standard python module `espp`
PYBIND11_MODULE(espp, m) {
#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif

  py_init_module_espp(m);
  py_init_cdr(m);
  py_init_rtps(m);
}
