idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES base_component esp_tinyusb
)

# X-Input registers a custom TinyUSB application class driver by overriding the
# weak usbd_app_driver_get_cb(). Because TinyUSB's usbd.c both defines that weak
# symbol AND calls it in the same translation unit, our strong override in
# usb_device.cpp is only guaranteed to win if the linker is forced to resolve the
# symbol globally. `-u` enters it as an undefined reference so the linker binds it
# to the strong definition (and patches usbd.c's call to it). Without this, the
# X-Input interface enumerates but its endpoints are never opened, so no
# controller input reaches the host.
target_link_options(${COMPONENT_LIB} INTERFACE "-u" "usbd_app_driver_get_cb")
