idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES driver esp_lcd fatfs base_component codec display display_drivers i2c input_drivers interrupt gt911 task bmi270 display_drivers ina226 pi4ioe5v rx8130ce
  REQUIRED_IDF_TARGETS "esp32p4"
  )
