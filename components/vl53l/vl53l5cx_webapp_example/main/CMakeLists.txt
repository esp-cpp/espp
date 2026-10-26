idf_component_register(
  SRC_DIRS "."
  INCLUDE_DIRS "."
  REQUIRES vl53l i2c usb_device dispatcher stream_frame task logger coredump monitor ota system espcoredump)
