idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES base_component interrupt filters led math mt6701 pid task bldc_driver bldc_motor i2c adc
  )
