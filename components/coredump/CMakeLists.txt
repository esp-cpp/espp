# Header-only component (coredump.hpp + coredump_service.hpp).
#
# REQUIRES are public since the headers include theirs:
#   espcoredump   — esp_core_dump.h (image check / summary / image get / erase)
#   esp_partition — esp_partition.h (raw image reads)
#   esp_system    — esp_system.h (esp_reset_reason)
#   format        — format.hpp (fmt::format in the report formatting)
#   ota           — detail/ota_stream_protocol.hpp (the shared stream framing
#                   used by coredump_service.hpp; header-only, ESP-free)
idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES base_component espcoredump esp_partition esp_system format ota
)
