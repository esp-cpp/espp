idf_build_get_property(idf_version IDF_VERSION)

# sdmmc: sdmmc_card_t / sdmmc_card_init; the SD host drivers and spi_master
# (SDSPI) are named in the public header; fatfs: the FAT mount / diskio calls.
set(requires base_component sdmmc fatfs)
# The SD host / SPI drivers moved out of the umbrella `driver` component in
# ESP-IDF v5.4.
if(idf_version VERSION_LESS "5.4")
  list(APPEND requires driver)
else()
  list(APPEND requires esp_driver_sdspi esp_driver_sdmmc esp_driver_spi)
endif()

idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES ${requires}
)
