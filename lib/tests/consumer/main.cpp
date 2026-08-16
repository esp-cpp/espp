// Smoke test: a minimal EXTERNAL consumer of the espp cross-platform C++ library.
// Verifies that the `espp::espp` target exposes the public include dirs and links
// the static library, regardless of how it was obtained (find_package /
// FetchContent / CPM). Driven by .github/workflows/cmake_consumer.yml.
//
// fast_math.hpp uses M_PI in non-template code, so compiling it on MSVC requires
// _USE_MATH_DEFINES to be defined before <cmath> - which the exported espp::espp
// target must propagate as a PUBLIC usage requirement (see lib/CMakeLists.txt).
// Including it here is what makes the windows matrix leg actually exercise that.
#include "fast_math.hpp"
#include "logger.hpp"

int main() {
  espp::Logger logger({.tag = "consumer", .level = espp::Logger::Verbosity::INFO});
  const float rad = espp::deg_to_rad(180.0f); // ~= pi; uses M_PI internally
  logger.info("espp consumer OK; deg_to_rad(180) = {}", rad);
  return (rad > 3.1f && rad < 3.2f) ? 0 : 1;
}
