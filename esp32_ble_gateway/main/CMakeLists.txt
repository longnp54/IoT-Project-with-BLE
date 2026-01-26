idf_component_register(SRCS "main_gateway.c" "thingsboard.c"
                    INCLUDE_DIRS "." "include"
                    REQUIRES nvs_flash esp_wifi esp_netif mqtt bt driver)
