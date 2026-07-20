idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_eth esp_event esp_netif driver lwip esp_timer u8g2 u8g2-hal-esp-idf esp_http_server app_update esp_partition esp_system freertos vfs lwip mac_hostname
)
