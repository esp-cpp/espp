// Smoke test: a minimal EXTERNAL consumer of the espp cross-platform C++ library.
// Verifies that the `espp::espp` target exposes the public include dirs and links
// the static library, regardless of how it was obtained (find_package /
// FetchContent / CPM). Driven by .github/workflows/cmake_consumer.yml.
//
// Uses espp::Logger (and, transitively, the vendored fmt) so a broken include
// path *or* a link failure against libespp_pc.a surfaces here.
#include "logger.hpp"

int main() {
  espp::Logger logger({.tag = "consumer", .level = espp::Logger::Verbosity::INFO});
  logger.info("espp consumer OK: espp::espp include + link resolved ({} + {})", 40, 2);
  return 0;
}
