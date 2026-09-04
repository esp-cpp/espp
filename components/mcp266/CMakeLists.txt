# NOTE: like canopen, this component's detail/ lives INSIDE include/
# (include/detail/mcp266_core.hpp), so registering "include" alone makes
# `#include "detail/mcp266_core.hpp"` resolve for consumers.
idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES base_component canopen motor_controller
)
