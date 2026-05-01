idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver esp_driver_i2s esp_driver_spi adc base_component bm8563 i2c interrupt led math task
  REQUIRED_IDF_TARGETS "esp32"
  )
