# only register the component if the target is esp32s3
idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver base_component display display_drivers i2c input_drivers interrupt gt911 task t_keyboard
  REQUIRED_IDF_TARGETS "esp32s3"
  )
