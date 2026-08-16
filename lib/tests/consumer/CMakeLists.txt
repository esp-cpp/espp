# Minimal external consumer of the espp cross-platform C++ library, used as a
# packaging smoke test (see ../../../.github/workflows/cmake_consumer.yml). One
# project that can obtain espp three ways so all supported consumption paths stay
# green:
#   -DESPP_CONSUMER_METHOD=find_package   (needs -DCMAKE_PREFIX_PATH=<install>)
#   -DESPP_CONSUMER_METHOD=fetchcontent   (needs -DESPP_SOURCE_DIR=<espp repo root>)
#   -DESPP_CONSUMER_METHOD=cpm            (needs -DESPP_SOURCE_DIR + -DCPM_espp_SOURCE=<espp repo root>)
# 3.24 for parity with the espp targets (the WHOLE_ARCHIVE genex lives in pc/).
cmake_minimum_required(VERSION 3.24)
project(espp_consumer LANGUAGES CXX)

set(ESPP_CONSUMER_METHOD "find_package" CACHE STRING "How to obtain espp: find_package | fetchcontent | cpm")
set(ESPP_SOURCE_DIR "" CACHE PATH "espp repo root (for fetchcontent / cpm)")

if(ESPP_CONSUMER_METHOD STREQUAL "find_package")
  find_package(espp REQUIRED)
elseif(ESPP_CONSUMER_METHOD STREQUAL "fetchcontent")
  include(FetchContent)
  # SOURCE_DIR consumes the local checkout (with its recursive submodules) rather
  # than re-cloning; a real consumer would use GIT_REPOSITORY/GIT_TAG instead.
  FetchContent_Declare(espp SOURCE_DIR "${ESPP_SOURCE_DIR}" SOURCE_SUBDIR lib)
  FetchContent_MakeAvailable(espp)
elseif(ESPP_CONSUMER_METHOD STREQUAL "cpm")
  include("${ESPP_SOURCE_DIR}/lib/cmake/CPM.cmake")
  # -DCPM_espp_SOURCE=<root> on the command line points CPM at the local checkout.
  CPMAddPackage(NAME espp GITHUB_REPOSITORY esp-cpp/espp GIT_TAG main SOURCE_SUBDIR lib)
else()
  message(FATAL_ERROR "unknown ESPP_CONSUMER_METHOD '${ESPP_CONSUMER_METHOD}' (find_package | fetchcontent | cpm)")
endif()

add_executable(espp_consumer main.cpp)
target_link_libraries(espp_consumer PRIVATE espp::espp)
