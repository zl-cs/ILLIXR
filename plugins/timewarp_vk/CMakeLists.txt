cmake_minimum_required(VERSION 3.16)

set(PLUGIN_NAME plugin.timewarp_vk${ILLIXR_BUILD_SUFFIX})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Vulkan COMPONENTS glslc)
# print glslc path
message(STATUS "glslc: ${glslc_executable}")

find_package(glm REQUIRED)

find_program(glslc_executable NAMES glslc)

file(GLOB_RECURSE GLSL_SOURCE_FILES
        "shaders/*.frag"
        "shaders/*.vert"
        )

foreach (GLSL ${GLSL_SOURCE_FILES})
    get_filename_component(FILE_NAME ${GLSL} NAME)
    set(SPIRV "${PROJECT_BINARY_DIR}/shaders/${FILE_NAME}.spv")
    add_custom_command(
            OUTPUT ${SPIRV}
            COMMAND ${CMAKE_COMMAND} -E make_directory "${PROJECT_BINARY_DIR}/shaders/"
            COMMAND ${glslc_executable} ${GLSL} -o ${SPIRV}
            DEPENDS ${GLSL})
    list(APPEND SPIRV_BINARY_FILES ${SPIRV})
endforeach (GLSL)

add_custom_target(
        Timewarp_vk_Shaders
        DEPENDS ${SPIRV_BINARY_FILES}
)

set(TIMEWARP_VK_SOURCES plugin.cpp
        utils/hmd.cpp
        utils/hmd.hpp
        ../../include/illixr/plugin.hpp
        ../../include/illixr/data_format.hpp
        ../../include/illixr/error_util.hpp
        ../../include/illixr/global_module_defs.hpp
        ../../include/illixr/math_util.hpp
        ../../include/illixr/phonebook.hpp
        ../../include/illixr/pose_prediction.hpp
        ../../include/illixr/switchboard.hpp
        ../../include/illixr/vk_util/display_sink.hpp
        ../../include/illixr/vk_util/render_pass.hpp
        ../../include/illixr/vk_util/vulkan_utils.hpp
        )

add_library(${PLUGIN_NAME} SHARED ${TIMEWARP_VK_SOURCES})
add_dependencies(${PLUGIN_NAME} Timewarp_vk_Shaders)
set_target_properties(${PLUGIN_NAME} PROPERTIES OUTPUT_NAME "plugin")

target_include_directories(${PLUGIN_NAME} PUBLIC ${Vulkan_INCLUDE_DIRS} ${ILLIXR_SOURCE_DIR}/include)
target_link_libraries(${PLUGIN_NAME} ${Vulkan_LIBRARIES})

add_custom_command(TARGET ${PLUGIN_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${PLUGIN_NAME}>/shaders/"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_LIST_DIR}/shaders"
        "$<TARGET_FILE_DIR:${PLUGIN_NAME}>/shaders"
        )

install(TARGETS ${PLUGIN_NAME} DESTINATION lib)
