cmake_minimum_required(VERSION 3.17)

project(offload_rendering_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(offload_rendering_server SHARED plugin.cpp)
set_target_properties(offload_rendering_server PROPERTIES OUTPUT_NAME "plugin")

target_include_directories(offload_rendering_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})