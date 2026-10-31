idf_component_register(
  SRC_DIRS "."
  INCLUDE_DIRS "."
  # canopen: the CiA 301 wire core the simulated node is built on
  REQUIRES usb_device twai dispatcher stream_frame task logger canopen coredump monitor ota system espcoredump)
