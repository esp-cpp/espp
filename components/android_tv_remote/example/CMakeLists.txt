cmake_minimum_required(VERSION 3.20)

# ---------------------------------------------------------------------------
# UI selection.
#
# The example runs either over the serial console (default) or on a LilyGo
# T-Deck, using its screen + keyboard. Board selection changes the set of
# components built and the flash/PSRAM config, which IDF resolves before Kconfig
# runs, so it is a CMake flag rather than a Kconfig choice:
#
#   idf.py -DATV_BOARD=console build     # serial monitor (default)
#   idf.py -DATV_BOARD=tdeck   build     # LilyGo T-Deck (screen + keyboard)
#
# We list the exact espp component directories each mode needs (rather than the
# whole components/ tree) so unrelated components -- and the managed registry
# dependencies they would drag in -- are never discovered.
# ---------------------------------------------------------------------------
if(NOT DEFINED ATV_BOARD)
  set(ATV_BOARD "tdeck")
endif()

# The console dev board and the T-Deck are both ESP32-S3.
if(NOT DEFINED IDF_TARGET)
  set(IDF_TARGET esp32s3 CACHE STRING "ESP-IDF target")
endif()

# espp component directories shared by both modes.
set(_atv_dirs
  android_tv_remote wifi cli nvs base_component logger format task)

set(_atv_components
  "main esptool_py nvs_flash android_tv_remote wifi mdns esp_driver_usb_serial_jtag esp_driver_uart vfs")
set(_atv_sdkconfig "sdkconfig.defaults")

if(ATV_BOARD STREQUAL "console")
  # console-only; no extra components.
  message(STATUS "ATV_BOARD=console: enabling serial console UI")
elseif(ATV_BOARD STREQUAL "tdeck")
  # Full transitive espp closure for the T-Deck BSP.
  list(APPEND _atv_dirs
    t-deck t_keyboard base_peripheral codec display display_drivers led gps i2c
    input_drivers touch interrupt gt911 spi sx126x)
  set(_atv_components "${_atv_components} t-deck")
  set(_atv_sdkconfig "sdkconfig.defaults;sdkconfig.defaults.tdeck")
  add_compile_definitions(ATV_UI_TDECK=1)
  message(STATUS "ATV_BOARD=tdeck: enabling T-Deck UI (screen + keyboard)")
else()
  message(FATAL_ERROR "Unknown ATV_BOARD='${ATV_BOARD}' (use console|tdeck)")
endif()

# Respect an explicit -DSDKCONFIG_DEFAULTS; otherwise use the per-mode list.
if(NOT DEFINED SDKCONFIG_DEFAULTS)
  set(SDKCONFIG_DEFAULTS "${_atv_sdkconfig}")
endif()

set(EXTRA_COMPONENT_DIRS "")
foreach(_d ${_atv_dirs})
  list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../../../components/${_d}")
endforeach()

set(COMPONENTS "${_atv_components}" CACHE STRING "List of components to include" FORCE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(android_tv_remote_example)

set(CMAKE_CXX_STANDARD 20)
