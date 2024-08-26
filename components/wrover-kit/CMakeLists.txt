idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver base_component display display_drivers task
  REQUIRED_IDF_TARGETS "esp32"
  )
