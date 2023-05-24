cmake_minimum_required(VERSION 3.17)

project(plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS sr_input.proto)

add_library(plugin SHARED plugin.cpp ${PROTO_SRCS} ${PROTO_HDRS})

#target_include_directories(plugin PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

#PROTOBUF_TARGET_CPP(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR} ${protobuf_files})

target_link_libraries(${PROJECT_NAME} 
    # eCAL::core
  protobuf::libprotobuf
)
