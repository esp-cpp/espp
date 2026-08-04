#include "espp.hpp"

// The Windows system libraries this needs (Ws2_32, winmm) are linked via CMake
// (see lib/espp.cmake, which sets ESPP_EXTERNAL_LIBS for WIN32), so no
// #pragma comment(lib, ...) is needed here - keeping the link spec in one place
// keeps it consistent across the static library, the tests, and the _espp
// Python module, and works on all Windows toolchains (not just MSVC).

#ifdef _WIN32
// Global instance that raises the multimedia timer resolution to 1 ms for the
// lifetime of the program (see TimerResolution in espp.hpp).
TimerResolution timer_resolution{};
#endif
