idf_component_register(
  SRC_DIRS "."
  # The CANopen app's simulated bus / DS402 node live in the can_bridge example
  # for now (simulated_can_bus.hpp, simulated_ds402_node.hpp); promoting them
  # into the canopen component is a follow-up.
  INCLUDE_DIRS "." "apps" "../../../canopen/can_bridge_example/main"
  # Every hardware component is required unconditionally (REQUIRES cannot
  # depend on Kconfig); the apps themselves are gated by
  # CONFIG_DESKTOP_EXAMPLE_ENABLE_* in desktop_example.cpp.
  REQUIRES desktop system monitor file_system nvs dispatcher task usb_device esp_tinyusb coredump ota espcoredump
           canopen twai i2c wifi ethernet)
