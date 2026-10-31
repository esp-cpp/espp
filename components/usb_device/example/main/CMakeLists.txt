idf_component_register(
  SRC_DIRS "."
  INCLUDE_DIRS "."
  REQUIRES usb_device esp_tinyusb dispatcher task system monitor ota coredump espcoredump fatfs
)
