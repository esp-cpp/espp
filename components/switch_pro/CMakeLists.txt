idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES base_component hid-rp timer
  PRIV_REQUIRES esp_hw_support esp_timer)
