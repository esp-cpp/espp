idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES driver led lvgl base_component task)
add_definitions(-DLV_CONF_INCLUDE_SIMPLE)
include_directories(include)
