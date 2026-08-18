# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

# NOTE: the IDF component manager is intentionally left ENABLED here (unlike most
# espp examples) so that it can fetch the managed `espressif/esp_tinyusb`
# component from the ESP registry. To avoid the component manager scanning every
# espp component manifest (some board components declare target-specific
# constraints that would fail on esp32s3), EXTRA_COMPONENT_DIRS is narrowed to
# just the components this example uses, and the espp dependencies are pinned to
# their in-repo copies via override_path in main/idf_component.yml.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add only the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../../../components/base_component"
  "../../../components/format"
  "../../../components/hid-rp"
  "../../../components/logger"
  "../../../components/odrive_ascii"
  "../../../components/usb_device"
)

set(
  COMPONENTS
  "main esptool_py base_component format hid-rp logger odrive_ascii usb_device esp_tinyusb"
  CACHE STRING
  "List of components to include"
  )

project(usb_cdc_example)

set(CMAKE_CXX_STANDARD 20)
