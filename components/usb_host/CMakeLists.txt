# The full-speed-only root port option touches the USB-DWC host controller
# registers through the HAL's low-level header, which lives in `esp_hal_usb`
# on IDF >= 6 and in `hal` before that.
set(usb_host_hal_component hal)
if(EXISTS "$ENV{IDF_PATH}/components/esp_hal_usb")
  set(usb_host_hal_component esp_hal_usb)
endif()

idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES base_component task usb usb_host_hid
  PRIV_REQUIRES ${usb_host_hal_component} soc
)
