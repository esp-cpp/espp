idf_component_register(
  SRCS "src/ping.cpp"
  INCLUDE_DIRS "include"
  REQUIRES "base_component" "cli"
  PRIV_REQUIRES "esp_netif" "esp_wifi" "esp_event" "esp_timer" "lwip"
)
