cmake_minimum_required(VERSION 3.2)
project(unreal_plugin)

add_library(json SHARED json.hpp)
set_target_properties(json PROPERTIES LINKER_LANGUAGE CXX)

add_definitions(-Wall -Wextra -Werror -Wno-format-extra-args -std=c++17 -g -O3)
add_library(plugin SHARED plugin.cpp)
target_link_libraries(plugin PUBLIC ${DEPENDENCIES} PRIVATE json)