idf_component_register(SRCS "ESP32_BT2AM.c"
                       PRIV_REQUIRES spi_flash nvs_flash esp_driver_gpio esp_timer bt
                       INCLUDE_DIRS "")
