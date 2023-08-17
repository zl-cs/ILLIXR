# module to build and install the gtsam_integrator ILLIXR plugin

set(PLUGIN_NAME plugin.gtsam_integrator${ILLIXR_BUILD_SUFFIX})

# source files, listed individually so that any changes will trigger a rebuild
add_library(${PLUGIN_NAME} SHARED plugin.cpp
        ../../include/illixr/plugin.hpp
        ../../include/illixr/data_format.hpp
        ../../include/illixr/switchboard.hpp
        )

# if GTSAM is being built from source
if(NOT GTSAM_FOUND)
    add_dependencies(${PLUGIN_NAME} ${GTSAM_DEP_STR})
endif()

# if building on CentOS make sure we use the correct OpenCV
if(BUILD_OPENCV)
    add_dependencies(${PLUGIN_NAME} OpenCV_Viz)
endif()

#add_definitions(-Wall -Wextra -Werror)
target_include_directories(${PLUGIN_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${GTSAM_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${ILLIXR_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
target_link_libraries(${PLUGIN_NAME} PUBLIC ${GTSAM_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen3_LIBRARIES})

if(BUILD_TESTS)
    add_executable(test_${PLUGIN_NAME} test_plugin.cpp)
    target_include_directories(test_${PLUGIN_NAME} PRIVATE ${ILLIXR_SOURCE_DIR}/include)
    target_link_libraries(test_${PLUGIN_NAME} PRIVATE GTest::gtest_main ${PLUGIN_NAME})
    add_test_coverage_plugin(${PLUGIN_NAME})
endif()

install(TARGETS ${PLUGIN_NAME} DESTINATION lib)

