if (ESP_PLATFORM)
  # set component requirements for ESP32
  set(COMPONENT_REQUIRES "format esp_timer")
else()
  # set component requirements for generic
  set(COMPONENT_REQUIRES "format")
endif()

idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES ${COMPONENT_REQUIRES})
