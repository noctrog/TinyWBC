cmake_minimum_required(VERSION 3.12)

# Build variables
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

project("tiny_wbc")

add_subdirectory(src)
add_subdirectory(examples)
