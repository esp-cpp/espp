idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES base_component task)

# add compiler flags to enable rtti
target_compile_options(${COMPONENT_LIB} PRIVATE -frtti)
