idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES esp_eth esp_lcd esp_netif base_component display ethernet sdcard esp_lcd_qemu_rgb
  REQUIRED_IDF_TARGETS "esp32" "esp32c3" "esp32s3"
  )
