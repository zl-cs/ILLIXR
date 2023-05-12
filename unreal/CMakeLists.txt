cmake_minimum_required(VERSION 3.2)
project(unreal_plugin)

find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
# link_directories("${CMAKE_SOURCE_DIR}/protobuf-3.19.0/lib")
# link_libraries(${Protobuf_LIBRARIES})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS illixr_ipc.proto)

include_directories("${CMAKE_SOURCE_DIR}/../../draco/src")
include_directories("${CMAKE_SOURCE_DIR}/../../draco/build")
link_libraries("${CMAKE_SOURCE_DIR}/../../draco/build/libdraco.a")

add_library(json SHARED json.hpp)
set_target_properties(json PROPERTIES LINKER_LANGUAGE CXX)

add_definitions(-Wall -Wextra -Werror -Wno-format-extra-args -std=c++17 -g -O3)

# add_library(plugin SHARED plugin.cpp ipc_server.cpp illixr_ipc.pb.cc illixr_ipc.pb.h)
add_library(plugin SHARED plugin.cpp ipc_server.cpp ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(plugin PUBLIC ${DEPENDENCIES} PRIVATE ${Protobuf_LIBRARIES})