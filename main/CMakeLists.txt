idf_component_register(SRCS "PI2.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES mqtt driver esp_wifi nvs_flash)
