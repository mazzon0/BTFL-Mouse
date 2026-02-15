file(GLOB_RECURSE srcs "src/*.c")
idf_component_register(SRCS ${srcs}
                       REQUIRES nvs_flash hogp tmx pmw3389dm esp_adc)
