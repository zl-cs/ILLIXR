cmake_minimum_required(VERSION 3.17)

project(offload_rendering_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)

add_library(offload_rendering_server SHARED plugin.cpp)
set_target_properties(offload_rendering_server PROPERTIES OUTPUT_NAME "plugin")

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIRS})
target_link_libraries(offload_rendering_server ${OpenCV_LIBS})
target_link_libraries(offload_rendering_server GLEW::GLEW)

target_include_directories(offload_rendering_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})