idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES
    "base_component"
    "driver"
    "ethernet"
    "esp_netif"
    "i2c"
  REQUIRED_IDF_TARGETS "esp32p4"
  )
