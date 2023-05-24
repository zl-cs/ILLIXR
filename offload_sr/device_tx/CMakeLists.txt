cmake_minimum_required(VERSION 3.17)

project(plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_BUILD_TYPE Release)

#find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)
find_package(OpenCV 3.4.6 EXACT REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
#add_library(plugin SHARED plugin.cpp video_encoder.cpp)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS sr_input.proto)
add_library(plugin SHARED plugin.cpp ${PROTO_SRCS} ${PROTO_HDRS})


#set(protobuf_files
#    ${CMAKE_CURRENT_SOURCE_DIR}/sr_input.proto
#)
#PROTOBUF_TARGET_CPP(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR} ${protobuf_files})

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  #eCAL::core
  protobuf::libprotobuf
)

#find_package(PkgConfig)
#pkg_search_module(GLIB REQUIRED glib-2.0)
#pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
#pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
#pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)
#pkg_check_modules(GST_AUDIO REQUIRED gstreamer-audio-1.0)
#
#include_directories(
#        /usr/lib64/glib-2.0/include
#        /usr/include/glib-2.0/
#        /usr/include/gstreamer-1.0
#        ${GLIB_INCLUDE_DIRS}
#)
#
#target_link_libraries(${PROJECT_NAME} ${GST_APP_LIBRARIES} ${GST_VIDEO_LIBRARIES} ${GST_AUDIO_LIBRARIES})
