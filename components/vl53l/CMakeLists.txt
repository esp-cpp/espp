idf_component_register(
  INCLUDE_DIRS "include"
  # src: the VL53L5CX driver and its firmware image (~84 KB of .rodata)
  SRC_DIRS "src"
  REQUIRES "base_peripheral"
  )
