idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver esp_driver_gpio esp_driver_spi esp_driver_i2c fatfs base_component task interrupt i2c spi gt911 bm8563 bq27220 pca9535 sx126x input_drivers lvgl epdiy
  REQUIRED_IDF_TARGETS "esp32s3"
  )
