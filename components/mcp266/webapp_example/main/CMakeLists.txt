idf_component_register(
  SRC_DIRS "."
  INCLUDE_DIRS "."
  REQUIRES mcp266 canopen twai usb_device dispatcher stream_frame task logger
)
