# Header-only, dependency-free wire-framing codec (CRC-32 framing + incremental
# StreamParser). No ESP-IDF component dependencies so it also builds on a host.
idf_component_register(
  INCLUDE_DIRS "include"
)
