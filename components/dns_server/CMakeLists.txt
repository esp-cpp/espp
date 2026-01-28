idf_component_register(
    SRCS "src/dns_server.cpp"
    INCLUDE_DIRS "include"
    REQUIRES base_component logger socket
)
