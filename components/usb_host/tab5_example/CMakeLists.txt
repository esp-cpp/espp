# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

# The component manager stays ON: the USB Host library (`usb`) and the HID class
# driver (`usb_host_hid`) come from the ESP Component Registry (on ESP-IDF >= 6.0
# `usb_host_hid` declares its `usb` dependency only through the manager), and the
# Tab5 BSP pulls its managed camera components. The espp components resolve
# locally from EXTRA_COMPONENT_DIRS.
set(ENV{IDF_COMPONENT_MANAGER} "1")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "../"
  "../../../components/base_component"
  "../../../components/base_peripheral"
  "../../../components/bmi270"
  "../../../components/cli"
  "../../../components/codec"
  "../../../components/display"
  "../../../components/display_drivers"
  "../../../components/esp-dsp"
  "../../../components/filters"
  "../../../components/format"
  "../../../components/gt911"
  "../../../components/hid-rp"
  "../../../components/i2c"
  "../../../components/ina226"
  "../../../components/input_drivers"
  "../../../components/interrupt"
  "../../../components/led"
  "../../../components/logger"
  "../../../components/lvgl"
  "../../../components/m5stack-tab5"
  "../../../components/math"
  "../../../components/pi4ioe5v"
  "../../../components/rx8130ce"
  "../../../components/sdcard"
  "../../../components/spi"
  "../../../components/st7123touch"
  "../../../components/task"
  "../../../components/timer"
  "../../../components/touch"
)

set(
  COMPONENTS
  "main esptool_py m5stack-tab5 display lvgl hid-rp usb_host usb usb_host_hid"
  CACHE STRING
  "List of components to include"
  )

project(usb_host_tab5_example)

set(CMAKE_CXX_STANDARD 20)
