# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add only the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../../../components/base_component"
  "../../../components/format"
  "../../../components/logger"
  "../../../components/task"
  "../../../components/timer"
  "../../../components/usb_host"
)

# The USB Host library (`usb`) and the HID class driver (`usb_host_hid`) are
# fetched from the ESP Component Registry by the IDF component manager (which is
# enabled by default). On ESP-IDF >= 6.0 the `usb_host_hid` component declares
# its `usb` dependency only through the manager, so unlike the device-side USB
# examples this one is built with the component manager **on** (the espp/*
# components are still resolved locally via EXTRA_COMPONENT_DIRS above).

set(
  COMPONENTS
  "main esptool_py base_component format logger task timer usb_host usb usb_host_hid"
  CACHE STRING
  "List of components to include"
  )

project(usb_host_example)

set(CMAKE_CXX_STANDARD 20)
