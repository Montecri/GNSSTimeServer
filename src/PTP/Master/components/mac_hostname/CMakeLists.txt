idf_component_register(
    SRCS "mac_hostname.c"
         "mac_hostname_esp.c"
         "name_data.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES esp_netif
    PRIV_REQUIRES esp_hw_support log)
