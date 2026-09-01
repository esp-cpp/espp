# Header-only component (coredump.hpp + coredump_service.hpp).
#
# REQUIRES are public since the headers include theirs:
#   espcoredump   — esp_core_dump.h (image check / summary / image get / erase)
#   esp_partition — esp_partition.h (raw image reads)
#   esp_system    — esp_system.h (esp_reset_reason)
#   format        — format.hpp (fmt::format in the report formatting)
#   stream_frame  — stream_frame.hpp (the shared frame codec used by
#                   coredump_service.hpp; header-only, ESP-free)
idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES base_component espcoredump esp_partition esp_system format stream_frame
)
