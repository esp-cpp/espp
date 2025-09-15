set(CLI_INCLUDES "include" "detail/cli/include")

idf_component_register(
  INCLUDE_DIRS ${CLI_INCLUDES}
  SRC_DIRS "src"
  REQUIRES driver esp_driver_uart esp_driver_usb_serial_jtag vfs logger)
