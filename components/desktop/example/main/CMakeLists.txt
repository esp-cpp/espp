# The hardware components are required unconditionally (REQUIRES cannot depend
# on Kconfig); the apps themselves are gated by CONFIG_DESKTOP_EXAMPLE_ENABLE_*
# in desktop_example.cpp. The exception is the CANopen app: espp's twai
# component needs IDF >= 6.0, so canopen / twai are only required when the
# top-level CMakeLists enabled DESKTOP_EXAMPLE_CANOPEN (ON by default on IDF
# >= 6; read back as a build property because IDF evaluates REQUIRES in a
# separate script process), and the app is compiled only when
# DESKTOP_EXAMPLE_CANOPEN_AVAILABLE is defined.
idf_build_get_property(desktop_example_canopen DESKTOP_EXAMPLE_CANOPEN)
set(desktop_example_requires desktop system monitor file_system nvs dispatcher task usb_device
    esp_tinyusb coredump ota espcoredump i2c wifi ethernet)
set(desktop_example_include_dirs "." "apps")
if(desktop_example_canopen)
  list(APPEND desktop_example_requires canopen twai)
  # The CANopen app's simulated bus / DS402 node live in the can_bridge example
  # for now (simulated_can_bus.hpp, simulated_ds402_node.hpp); promoting them
  # into the canopen component is a follow-up.
  list(APPEND desktop_example_include_dirs "../../../canopen/can_bridge_example/main")
endif()

idf_component_register(
  SRC_DIRS "."
  INCLUDE_DIRS ${desktop_example_include_dirs}
  REQUIRES ${desktop_example_requires})

if(desktop_example_canopen)
  target_compile_definitions(${COMPONENT_LIB} PRIVATE DESKTOP_EXAMPLE_CANOPEN_AVAILABLE=1)
endif()
