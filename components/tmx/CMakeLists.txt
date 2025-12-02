file(GLOB_RECURSE srcs "src/*.c")

idf_component_register(SRCS "${srcs}"
                       REQUIRES touch_element
                       INCLUDE_DIRS "./include")
