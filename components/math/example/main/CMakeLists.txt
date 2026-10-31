idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS ".")

# Keep this example float-only (esp32 has no hardware double). Scoped to the
# example's own translation units so it does not reach ESP-IDF's components.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wdouble-promotion)
