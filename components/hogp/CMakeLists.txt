file(GLOB_RECURSE srcs "src/*.c")
list(REMOVE_ITEM srcs "${CMAKE_CURRENT_SOURCE_DIR}/test/test_serializer.c")

idf_component_register(SRCS "${srcs}"
                       PRIV_REQUIRES bt
                       INCLUDE_DIRS "./include"
                       PRIV_INCLUDE_DIRS "./src")