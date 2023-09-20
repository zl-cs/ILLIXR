# module to build and install the offload_data ILLIXR plugin

set(PLUGIN_NAME plugin.offload_data${ILLIXR_BUILD_SUFFIX})

# source files, listed individually so that any changes will trigger a rebuild
add_library(${PLUGIN_NAME} SHARED plugin.cpp
        ../../include/illixr/plugin.hpp
        ../../include/illixr/data_format.hpp
        ../../include/illixr/error_util.hpp
        ../../include/illixr/global_module_defs.hpp
        ../../include/illixr/phonebook.hpp
        ../../include/illixr/switchboard.hpp
        )

target_include_directories(${PLUGIN_NAME} PRIVATE ${Boost_INCLUDE_DIR} ${ILLIXR_SOURCE_DIR}/include ${Eigen3_INCLUDE_DIRS})
target_link_libraries(${PLUGIN_NAME} boost_filesystem ${Eigen3_LIBRARIES} spdlog::spdlog)
target_compile_features(${PLUGIN_NAME} PRIVATE cxx_std_17)

if(BUILD_TESTS)
    add_executable(test_${PLUGIN_NAME} test_plugin.cpp)
    target_include_directories(test_${PLUGIN_NAME} PRIVATE ${ILLIXR_SOURCE_DIR}/include)
    target_link_libraries(test_${PLUGIN_NAME} PRIVATE GTest::gtest_main ${PLUGIN_NAME})
    add_test_coverage_plugin(${PLUGIN_NAME})
endif()

install(TARGETS ${PLUGIN_NAME} DESTINATION lib)
