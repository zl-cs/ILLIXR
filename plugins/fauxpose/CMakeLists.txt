# module to build and install the fauxpose ILLIXR plugin

set(PLUGIN_NAME plugin.fauxpose${ILLIXR_BUILD_SUFFIX})

set(FAUXPOSE_SOURCES plugin.cpp
        ../../include/illixr/plugin.hpp
        ../../include/illixr/pose_prediction.hpp
        )

add_library(${PLUGIN_NAME} SHARED ${FAUXPOSE_SOURCES})
# if building on CentOS make sure we use the correct OpenCV
if(BUILD_OPENCV)
    add_dependencies(${PLUGIN_NAME} OpenCV_Viz)
endif()

target_include_directories(${PLUGIN_NAME} PRIVATE ${ILLIXR_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PLUGIN_NAME} PUBLIC ${OpenCV_LIBRARIES})
target_compile_features(${PLUGIN_NAME} PRIVATE cxx_std_17)

if(BUILD_TESTS)
    add_executable(test_${PLUGIN_NAME} test_plugin.cpp)
    target_include_directories(test_${PLUGIN_NAME} PRIVATE ${ILLIXR_SOURCE_DIR}/include)
    target_link_libraries(test_${PLUGIN_NAME} PRIVATE GTest::gtest_main ${PLUGIN_NAME})
    add_test_coverage_plugin(${PLUGIN_NAME})
endif()

install(TARGETS ${PLUGIN_NAME} DESTINATION lib)
