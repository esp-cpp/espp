# only register the component if the target is esp32s3
idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver esp_adc esp_driver_i2s fatfs base_component adc display display_drivers i2c interrupt led neopixel spi task
  REQUIRED_IDF_TARGETS "esp32s3"
  )
