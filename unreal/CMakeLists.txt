cmake_minimum_required(VERSION 3.2)
project(unreal_plugin)

find_package(Protobuf REQUIRED)

add_library(json SHARED json.hpp)
set_target_properties(json PROPERTIES LINKER_LANGUAGE CXX)

add_definitions(-Wall -Wextra -Werror -Wno-format-extra-args -std=c++17 -g -O3)

configure_file(mesh.objp mesh.objp COPYONLY)

include_directories(${Protobuf_INCLUDE_DIRS})

add_library(plugin SHARED plugin.cpp ipc_server.cpp illixr_ipc.pb.cc illixr_ipc.pb.h)
target_link_libraries(plugin PUBLIC ${DEPENDENCIES} PRIVATE ${Protobuf_LIBRARIES})