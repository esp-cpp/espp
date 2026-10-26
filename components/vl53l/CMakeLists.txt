# vl53l.hpp / vl53l5cx.hpp need base_peripheral. The optional
# vl53l5cx_service.hpp (the VL53L5CX console protocol as a dispatcher service)
# adds base_component / dispatcher / stream_frame; vl53l5cx_protocol.hpp is
# standard-library only.
idf_component_register(
  INCLUDE_DIRS "include"
  # src: the VL53L5CX driver and its firmware images (~84 KB of .rodata each)
  SRC_DIRS "src"
  REQUIRES "base_peripheral" "base_component" "dispatcher" "stream_frame"
  )
