idf_component_register(
    SRCS
        "main.c"
        "allystar_timing.c"
    INCLUDE_DIRS "."
    REQUIRES
        # networking stack
        esp_eth esp_event esp_netif lwip
        # storage / OTA
        app_update esp_partition
        # system runtime
        esp_system esp_timer freertos vfs
        # peripherals & display
        driver u8g2 u8g2-hal-esp-idf
        # web
        esp_http_server
        # local components
        mac_hostname
)