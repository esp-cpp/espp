# NOTE: like odrive_native / canopen, this component's detail/ lives INSIDE
# include/ (include/detail/ota_stream_protocol.hpp), so registering "include"
# alone makes `#include "detail/ota_stream_protocol.hpp"` resolve for consumers.
#
# REQUIRES are public since ota.hpp (header-only) includes their headers:
#   app_update         — esp_ota_ops.h
#   esp_app_format     — esp_app_desc.h (esp_app_desc_t)
#   bootloader_support — esp_app_format.h (esp_image_header_t / magic 0xE9)
#   esp_partition      — esp_partition.h
idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES base_component app_update esp_app_format bootloader_support esp_partition
)
