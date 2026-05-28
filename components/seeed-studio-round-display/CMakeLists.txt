idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver base_component chsc6x display display_drivers i2c input_drivers interrupt spi task
  REQUIRED_IDF_TARGETS "esp32s3"
  )
