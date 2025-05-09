# Determine whether file_system is fetched from component registry or from local path
idf_build_get_property(build_components BUILD_COMPONENTS)
if(file_system IN_LIST build_components)
    set(required_littlefs_name esp_littlefs) # Local component
else()
    set(required_littlefs_name littlefs) # Managed component
endif()


idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES base_component ${required_littlefs_name} spi_flash)
