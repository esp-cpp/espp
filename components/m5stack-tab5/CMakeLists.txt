# ESP-IDF 6.1 adds esp_driver_dma's async color-convert driver (DMA2D window
# copies), which the back-buffer flush uses when present (see video.cpp).
set(m5stack_tab5_extra_requires "")
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "6.1")
  list(APPEND m5stack_tab5_extra_requires esp_driver_dma)
endif()

idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver esp_driver_i2s esp_driver_ppa esp_driver_sdmmc esp_driver_spi esp_lcd esp_video esp_cam_sensor fatfs base_component bmi270 codec display display_drivers gt911 i2c ina226 input_drivers interrupt pi4ioe5v rx8130ce sdcard st7123touch task touch ${m5stack_tab5_extra_requires}
  REQUIRED_IDF_TARGETS "esp32p4"
  )
