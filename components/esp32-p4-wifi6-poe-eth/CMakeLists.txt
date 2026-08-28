idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES
    "base_component"
    "ethernet"
    "esp_netif"
    "i2c"
    "interrupt"
  REQUIRED_IDF_TARGETS "esp32p4"
  )
