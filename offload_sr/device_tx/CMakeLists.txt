cmake_minimum_required(VERSION 3.17)

project(offload_sr_device_tx)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_BUILD_TYPE Release)

find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR}) 


add_library(plugin SHARED plugin.cpp)

set(protobuf_files
    ${CMAKE_CURRENT_SOURCE_DIR}/sr_input.proto
)

PROTOBUF_TARGET_CPP(plugin ${CMAKE_CURRENT_SOURCE_DIR} ${protobuf_files})


target_link_libraries(plugin
  ${DEPENDENCIES}
  eCAL::core
  ${Protobuf_LIBRARIES}
)
