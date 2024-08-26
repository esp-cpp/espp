idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver base_component codec display display_drivers i2c input_drivers interrupt gt911 task tt21100
  REQUIRED_IDF_TARGETS "esp32s3"
  )
