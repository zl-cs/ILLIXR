
include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(test_cpu_timer tests/test_cpu_timer.cpp)
if(BUILD_OPENCV)
    add_dependencies(test_cpu_timer OpenCV_Viz)
endif()

target_include_directories(test_cpu_timer PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(test_cpu_timer PRIVATE GTest::gtest_main)

add_executable(test_switchboard tests/test_switchboard.cpp)
target_link_libraries(test_switchboard PRIVATE GTest::gtest_main)

add_test_coverage(test_cpu_timer)
add_test_coverage(test_switchboard)
