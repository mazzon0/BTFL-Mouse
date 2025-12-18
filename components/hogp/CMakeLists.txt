file(GLOB_RECURSE srcs "src/*.c")

idf_component_register(SRCS "${srcs}"
                       PRIV_REQUIRES bt
                       INCLUDE_DIRS "./include"
                       PRIV_INCLUDE_DIRS "./src")
