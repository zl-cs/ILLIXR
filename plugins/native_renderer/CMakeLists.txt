cmake_minimum_required(VERSION 3.16)

set(PLUGIN_NAME plugin.native_renderer${ILLIXR_BUILD_SUFFIX})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Vulkan COMPONENTS glslc)
find_package(glm REQUIRED)

set(NATIVE_RENDERER_SOURCES plugin.cpp
        ../../include/illixr/data_format.hpp
        ../../include/illixr/error_util.hpp
        ../../include/illixr/global_module_defs.hpp
        ../../include/illixr/math_util.hpp
        ../../include/illixr/pose_prediction.hpp
        ../../include/illixr/switchboard.hpp
        ../../include/illixr/threadloop.hpp
        ../../include/illixr/vk_util/display_sink.hpp
        ../../include/illixr/vk_util/render_pass.hpp
        ../../include/illixr/threadloop.hpp
        ../../include/illixr/gl_util/lib/tiny_obj_loader.h
        )
add_library(${PLUGIN_NAME} SHARED ${NATIVE_RENDERER_SOURCES})

# if building on CentOS make sure we use the correct OpenCV
if(BUILD_OPENCV)
    add_dependencies(${PLUGIN_NAME} OpenCV_Viz)
endif()

set_target_properties(${PLUGIN_NAME} PROPERTIES OUTPUT_NAME "plugin")

target_include_directories(${PLUGIN_NAME} PUBLIC ${Vulkan_INCLUDE_DIRS} ${ILLIXR_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${glm_INCLUDE_DIRS})
target_link_libraries(${PLUGIN_NAME} ${Vulkan_LIBRARIES} ${Eigen3_LIBRARIES} ${OpenCV_LIBRARIES} )

install(TARGETS ${PLUGIN_NAME} DESTINATION lib)
