idf_component_register(SRCS "main.c" "board.c" "eth.c" "wifi.c" "control.c" "console.c" "ota.c" "creds.c"
                       INCLUDE_DIRS "."
                       REQUIRES bridge_core esp_eth esp_wifi esp_event nvs_flash app_update esp_driver_gpio
                                esp_driver_uart esp_timer mbedtls esp_app_format)
