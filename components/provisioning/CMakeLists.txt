idf_component_register(
    SRCS "src/provisioning.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_http_server nvs wifi logger base_component
)
