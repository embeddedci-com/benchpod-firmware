set(srcs "main.c" "board.c" "eth.c" "wifi.c" "control.c" "console.c" "ota.c" "creds.c")
# EMC test build (./build.sh emc): development only, never with secure boot or flash encryption.
if(CONFIG_BRIDGE_EMC_TEST)
    if(CONFIG_SECURE_BOOT OR CONFIG_SECURE_FLASH_ENC_ENABLED)
        message(FATAL_ERROR "CONFIG_BRIDGE_EMC_TEST is a development build and cannot be combined "
                            "with the production overlay (secure boot, flash encryption)")
    endif()
    list(APPEND srcs "emc_test.c")
endif()
idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       REQUIRES bridge_core esp_eth esp_wifi esp_event nvs_flash app_update esp_driver_gpio
                                esp_driver_uart esp_timer esp_app_format)
