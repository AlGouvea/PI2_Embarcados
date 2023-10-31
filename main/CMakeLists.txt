idf_component_register(SRCS "PI2.c" "DHT.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES mqtt driver esp_wifi nvs_flash esp_event vfs esp_http_server)
