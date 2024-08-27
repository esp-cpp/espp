idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver adc base_component bm8563 i2c interrupt led math task
  REQUIRED_IDF_TARGETS "esp32"
  )
