# NOTE: like odrive_native, this component's detail/ lives INSIDE include/
# (include/detail/canopen_core.hpp), so registering "include" alone makes
# `#include "detail/canopen_core.hpp"` resolve for consumers.
idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES base_component
)
