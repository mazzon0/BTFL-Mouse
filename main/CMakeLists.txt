idf_component_register(
    SRCS "main.c"
    REQUIRES freertos driver esp_timer
    PRIV_REQUIRES pmw3389dm
)