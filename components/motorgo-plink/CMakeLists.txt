idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES base_component bdc_driver esp_driver_gpio i2c led lsm6dso math mt6701 spi task
  REQUIRED_IDF_TARGETS "esp32s3"
  )
