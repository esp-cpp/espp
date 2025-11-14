idf_component_register(
  INCLUDE_DIRS "include" "detail/cli/include"
  SRC_DIRS "src"
  REQUIRES driver esp_driver_uart esp_driver_usb_serial_jtag vfs esp_vfs_console logger)
