idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES lvgl logger task)

add_definitions(-DLV_CONF_INCLUDE_SIMPLE)
include_directories(include)
