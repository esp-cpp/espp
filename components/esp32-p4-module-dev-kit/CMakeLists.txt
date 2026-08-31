idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES
    "base_component"
    "ethernet"
    "esp_netif"
    "fatfs"
    "esp_driver_sdmmc"
    "sdmmc"
    "codec"
    "i2c"
    "task"
    "esp_driver_i2s"
    "display"
    "display_drivers"
    "gt911"
    "input_drivers"
    "interrupt"
    "esp_lcd"
    "esp_video"
    "esp_cam_sensor"
  REQUIRED_IDF_TARGETS "esp32p4"
  )

# This BSP uses the new espp/i2c API (I2c::Device, add_device(),
# native_bus_handle()), which is unavailable under CONFIG_ESPP_I2C_USE_LEGACY_API.
# Consumers of the published component do not inherit the example's
# sdkconfig.defaults, so surface the requirement as an explicit configure-time
# error instead of a confusing compile failure.
if(CONFIG_ESPP_I2C_USE_LEGACY_API)
  message(FATAL_ERROR
    "esp32-p4-module-dev-kit requires the new espp/i2c API. Set "
    "CONFIG_ESPP_I2C_USE_NEW_API=y (I2C API selection under the espp/i2c menu).")
endif()
