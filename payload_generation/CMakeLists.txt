cmake_minimum_required(VERSION 3.2)
project(payload_generation)

find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS illixr_ipc.proto)


add_definitions(-Wall -Wextra -Werror -Wno-format-extra-args -std=c++17 -g -O3)

# add_library(plugin SHARED plugin.cpp ipc_server.cpp illixr_ipc.pb.cc illixr_ipc.pb.h)
#add_library(plugin SHARED plugin.cpp ipc_server.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_library(plugin SHARED plugin.cpp ${PROTO_SRCS} ${PROTO_HDRS})
#target_link_libraries(plugin ${DRACO_LIB} ${DEPENDENCIES} ${Protobuf_LIBRARIES})
target_link_libraries(plugin ${DEPENDENCIES} ${Protobuf_LIBRARIES})
#target_link_libraries(plugin PUBLIC ${DEPENDENCIES} PRIVATE ${Protobuf_LIBRARIES})
