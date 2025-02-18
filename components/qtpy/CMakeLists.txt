# only register the component if the target is esp32s3 or esp32
idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES base_component i2c neopixel task interrupt
  REQUIRED_IDF_TARGETS "esp32s3" "esp32"
  )
