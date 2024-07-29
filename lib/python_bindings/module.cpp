#include <pybind11/pybind11.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

void py_init_module_espp_lib(py::module &m);

// This builds the native python module `_espp_lib`
// it will be wrapped in a standard python module `espp_lib`
PYBIND11_MODULE(espp, m) {
#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif

  py_init_module_espp_lib(m);
}
