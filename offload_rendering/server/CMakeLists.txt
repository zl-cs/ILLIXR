cmake_minimum_required(VERSION 3.17)

project(offload_rendering_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)

find_package(PkgConfig)
pkg_search_module(GLIB REQUIRED glib-2.0)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)
pkg_check_modules(GST_AUDIO REQUIRED gstreamer-audio-1.0)

add_library(offload_rendering_server SHARED plugin.cpp video_encoder.cpp)
set_target_properties(offload_rendering_server PROPERTIES OUTPUT_NAME "plugin")

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIRS})

include_directories(
        /usr/lib64/glib-2.0/include
        /usr/include/glib-2.0/
        /usr/include/gstreamer-1.0
        /usr/lib/x86_64-linux-gnu/gstreamer-1.0/include # ubuntu 20.04 fix
        ${GLIB_INCLUDE_DIRS}
)

target_link_libraries(offload_rendering_server ${OpenCV_LIBS})
target_link_libraries(offload_rendering_server GLEW::GLEW)
target_link_libraries(${PROJECT_NAME} ${GST_APP_LIBRARIES} ${GST_VIDEO_LIBRARIES} ${GST_AUDIO_LIBRARIES})

target_include_directories(offload_rendering_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})