# Header-only frame router. Depends only on the stream_frame codec (both are
# dependency-free, so this also builds on a host).
idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES stream_frame
)
