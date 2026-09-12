idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  # esp_hw_support (esp_mac.h / esp_random.h) and esp_timer (esp_timer.h) are
  # included by the PUBLIC switch_pro.hpp, so downstream consumers of that header
  # need their include paths -> public REQUIRES, not PRIV_REQUIRES.
  REQUIRES base_component hid-rp timer esp_hw_support esp_timer)
