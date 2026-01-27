idf_component_register(
    INCLUDE_DIRS "include"
    SRC_DIRS "src"
    REQUIRES esp_http_server driver adc base_component task timer file_system
)
