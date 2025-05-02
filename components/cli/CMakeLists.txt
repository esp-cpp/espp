set(CLI_INCLUDES "include")

if (CONFIG_COMPILER_CXX_EXCEPTIONS)
  list(APPEND CLI_INCLUDES "detail/cli/include")
else()
  message(WARNING "C++ exceptions are disabled, CLI is disabled")
endif()

idf_component_register(
  INCLUDE_DIRS ${CLI_INCLUDES}
  SRC_DIRS "src"
  REQUIRES driver esp_driver_uart esp_driver_usb_serial_jtag vfs logger)
