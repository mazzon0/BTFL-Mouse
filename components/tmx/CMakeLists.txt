file(GLOB_RECURSE srcs "src/*.c")

idf_component_register(SRCS "${srcs}"
                       REQUIRES bt nvs_flash
                       INCLUDE_DIRS "./include")
