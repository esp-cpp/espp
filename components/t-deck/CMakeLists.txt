# only register the component if the target is esp32s3
idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver esp_driver_i2s esp_driver_spi base_component codec display display_drivers fatfs gps i2c input_drivers interrupt gt911 spi sx126x task t_keyboard
  REQUIRED_IDF_TARGETS "esp32s3"
  )
