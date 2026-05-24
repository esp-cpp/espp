idf_component_register(
    INCLUDE_DIRS "include"
    SRC_DIRS "src"
    REQUIRES esp_http_server driver esp_driver_gpio adc base_component file_system timer
)
