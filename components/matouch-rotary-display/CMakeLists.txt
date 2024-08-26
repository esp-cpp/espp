# only register the component if the target is esp32s3
idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver base_component cst816 encoder display display_drivers i2c input_drivers interrupt task
  REQUIRED_IDF_TARGETS "esp32s3"
  )
