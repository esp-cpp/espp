# only register the component if the target is esp32s3
idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver base_component cst816 display display_drivers encoder i2c input_drivers interrupt spi task
  REQUIRED_IDF_TARGETS "esp32s3"
  )
