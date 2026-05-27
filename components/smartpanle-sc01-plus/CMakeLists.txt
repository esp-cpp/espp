idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver esp_driver_i2s esp_lcd fatfs sdmmc base_component display display_drivers ft5x06 i2c input_drivers interrupt led task
  REQUIRED_IDF_TARGETS "esp32s3"
  )
