idf_component_register(
    INCLUDE_DIRS "include"
    SRC_DIRS "src"
    REQUIRES base_component logger mqtt esp_http_client esp_websocket_client json
)
