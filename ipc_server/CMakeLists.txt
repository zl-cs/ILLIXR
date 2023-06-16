cmake_minimum_required(VERSION 3.2)
project(ipc_server)

#find_package(Protobuf REQUIRED)
#include_directories(${Protobuf_INCLUDE_DIRS})
#include_directories(${CMAKE_CURRENT_BINARY_DIR})
## link_directories("${CMAKE_SOURCE_DIR}/protobuf-3.19.0/lib")
## link_libraries(${Protobuf_LIBRARIES})
#protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS illixr_ipc.proto)


add_definitions(-Wall -Wextra -Werror -Wno-format-extra-args -std=c++17 -g -O3)
add_library(plugin SHARED plugin.cpp ipc_server.cpp)
target_link_libraries(plugin ${DEPENDENCIES})

#add_library(plugin SHARED plugin.cpp ipc_server.cpp ${PROTO_SRCS} ${PROTO_HDRS})
#target_link_libraries(plugin ${DEPENDENCIES} ${Protobuf_LIBRARIES})

