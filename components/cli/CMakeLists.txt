include($ENV{IDF_PATH}/tools/cmake/version.cmake)

set(cli_requires driver esp_driver_uart esp_driver_usb_serial_jtag vfs logger)

if(IDF_VERSION_MAJOR GREATER_EQUAL 6)
  list(APPEND cli_requires esp_stdio esp_usb_cdc_rom_console)
else()
  list(APPEND cli_requires esp_vfs_console)
endif()

idf_component_register(
  INCLUDE_DIRS "include" "detail/cli/include"
  SRC_DIRS "src"
  REQUIRES ${cli_requires})
