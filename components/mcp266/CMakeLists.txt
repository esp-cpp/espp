# NOTE: like canopen, this component's detail/ lives INSIDE include/
# (include/detail/mcp266_core.hpp), so registering "include" alone makes
# `#include "detail/mcp266_core.hpp"` resolve for consumers.
#
# mcp266.hpp needs base_component + canopen + motor_controller. The optional
# mcp266_service.hpp (the console protocol as a dispatcher service) adds
# dispatcher / stream_frame / task; mcp266_protocol.hpp is standard-library only.
idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES base_component canopen motor_controller dispatcher stream_frame task
)
