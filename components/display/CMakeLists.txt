idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES lvgl logger task)
# lvgl generates deprecated enum-enum conversion warnings, suppress them
target_compile_options(${COMPONENT_LIB} INTERFACE "-Wno-deprecated-enum-enum-conversion")
add_definitions(-DLV_CONF_INCLUDE_SIMPLE)
include_directories(include)
