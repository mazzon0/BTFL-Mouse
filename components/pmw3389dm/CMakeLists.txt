file(GLOB_RECURSE srcs "src/*.c")

idf_component_register(
    SRCS "src/pmw3389.c" "src/pmw3389_srom.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer freertos
    PRIV_REQUIRES esp_rom
)