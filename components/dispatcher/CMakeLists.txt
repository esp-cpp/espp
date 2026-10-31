# dispatcher.hpp is header-only and depends only on stream_frame (it builds and
# unit-tests on a host). dispatcher_worker.hpp adds the worker-task variant and
# pulls in task + base_component.
idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES stream_frame task base_component
)
