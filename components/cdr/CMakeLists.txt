# The cdr library (detail/cdr, github.com/finger563/cdr) is header-only; its
# reflection backend comes from the reflect_cpp component. Interface-only
# registration, like the serialization component.
idf_component_register(
  INCLUDE_DIRS "detail/cdr/include" "include"
  REQUIRES reflect_cpp)
