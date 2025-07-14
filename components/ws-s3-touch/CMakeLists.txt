idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver base_component display display_drivers i2c input_drivers interrupt led cst816 task qmi8658
  REQUIRED_IDF_TARGETS "esp32s3"
  )
