idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES base_component esp_eth esp_netif esp_event esp_driver_spi esp_driver_gpio esp_hw_support)
