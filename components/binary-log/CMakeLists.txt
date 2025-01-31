idf_component_register(
  INCLUDE_DIRS "include" "detail/include"
  SRC_DIRS "src"
  REQUIRES "")

# idf_build_get_property(BINARY_NAME EXECUTABLE)

add_custom_command(
  TARGET ${COMPONENT_TARGET}
  POST_BUILD
  DEPENDS ${COMPONENT_TARGET}
  WORKING_DIRECTORY ${COMPONENT_DIR}/detail
  COMMAND echo "Building binary-log unpacker"
  COMMAND mkdir -p build
  COMMAND cmake -B build -S .
  COMMAND cmake --build build
  COMMAND cp build/tools/unpacker/unpacker ${CMAKE_BINARY_DIR}/unpacker
)
