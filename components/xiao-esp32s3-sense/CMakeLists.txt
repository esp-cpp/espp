idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES base_component driver esp_driver_gpio esp_driver_i2s fatfs led math task
  REQUIRED_IDF_TARGETS "esp32s3"
  )
