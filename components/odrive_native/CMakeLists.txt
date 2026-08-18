# NOTE: unlike the vendored root-level detail/ folders in other espp components
# (format, cdr, hid-rp, ...), this component's detail/ lives INSIDE include/
# (include/detail/*.hpp), so registering "include" alone makes
# `#include "detail/odrive_native_core.hpp"` resolve for consumers.
idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES base_component
)
