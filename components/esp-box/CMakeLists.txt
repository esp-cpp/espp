idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver audio base_component display display_drivers i2c input_drivers interrupt gt911 spi task touch tt21100 icm42607
  REQUIRED_IDF_TARGETS "esp32s3"
  )
