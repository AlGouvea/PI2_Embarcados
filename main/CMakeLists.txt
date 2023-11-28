idf_component_register(
    SRCS "PI2.c" "DHT/DHT.c" "cJSON/cJSON.c"
    INCLUDE_DIRS "." "cJSON" "DHT"
    PRIV_REQUIRES mqtt driver esp_wifi nvs_flash esp_event vfs esp_http_server esp_timer
)
