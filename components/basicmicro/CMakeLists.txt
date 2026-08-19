# NOTE: like odrive_native, this component's detail/ lives INSIDE include/
# (include/detail/*.hpp) so that the host-buildable wire core can be included
# as `#include "detail/basicmicro_core.hpp"` by consumers and by the host test.
idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES base_component
)
