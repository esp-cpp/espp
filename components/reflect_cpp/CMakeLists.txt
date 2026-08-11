# reflect-cpp's core reflection (rfl/to_view.hpp and friends) is header-only;
# the compiled library / bundled yyjson exist only for its JSON backend, which
# nothing here uses — so this component is include-dirs only.
idf_component_register(
  INCLUDE_DIRS "detail/reflect-cpp/include")
