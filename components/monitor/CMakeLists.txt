# stream_frame + dispatcher: the header-only MonitorService (monitor_service.hpp)
# that serves the heap / task statistics over a framed byte stream.
idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES base_component task stream_frame dispatcher)
