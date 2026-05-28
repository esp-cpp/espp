idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES adc base_component bldc_driver bldc_motor esp_driver_spi filters i2c interrupt led math mt6701 pid spi task
  REQUIRED_IDF_TARGETS "esp32s3"
  )
