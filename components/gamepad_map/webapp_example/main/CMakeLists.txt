idf_component_register(
  SRC_DIRS "."
  INCLUDE_DIRS "."
  REQUIRES gamepad_map file_system usb_device dispatcher stream_frame task logger coredump monitor ota system espcoredump esp_driver_gpio)
