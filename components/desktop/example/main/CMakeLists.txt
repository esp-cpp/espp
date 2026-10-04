idf_component_register(
  SRC_DIRS "."
  INCLUDE_DIRS "." "apps"
  REQUIRES desktop system monitor file_system nvs dispatcher task usb_device esp_tinyusb coredump ota espcoredump)
