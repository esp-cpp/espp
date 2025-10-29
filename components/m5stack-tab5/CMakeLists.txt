idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver esp_lcd fatfs base_component bmi270 codec display display_drivers gt911 i2c ina226 input_drivers interrupt pi4ioe5v rx8130ce task
  REQUIRED_IDF_TARGETS "esp32p4"
  )
