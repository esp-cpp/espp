# NOTE: espcoredump must be REQUIRED explicitly (not just transitively through
# the espp coredump component) so its header + Kconfig symbols
# (CONFIG_ESP_COREDUMP_*) are visible to this component on IDF 6.
idf_component_register(
  SRC_DIRS "."
  INCLUDE_DIRS "."
  REQUIRES coredump dispatcher task usb_device esp_tinyusb espcoredump esp_driver_gpio
)
