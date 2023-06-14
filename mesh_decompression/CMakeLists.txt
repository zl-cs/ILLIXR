##########################################
# Top-level CMakeLists.txt for InfiniTAM #
##########################################

cmake_minimum_required(VERSION 3.0)

PROJECT(MeshDecompression)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(Threads REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV 3.4.6 EXACT REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})

find_library(DRACO_LIB draco HINTS /usr/local/lib) 

# Prefer GLVND
if (POLICY CMP0072)
  cmake_policy (SET CMP0072 NEW)
endif(POLICY CMP0072)

set(CMAKE_C_COMPILER "/usr/bin/clang")
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

#################################
# Specify the CMake module path #
#################################

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

#################################
# Add additional compiler flags #
#################################

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -std=c++17 -Wno-deprecated-declarations -Wno-unused-function -I /usr/local/cuda-11.2/include")
ENDIF()

IF(NOT MSVC_IDE)
  SET(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  SET(CMAKE_CXX_FLAGS "-fPIC -O3 -std=c++17 -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
  #SET(CMAKE_CXX_FLAGS "-fPIC -g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
ENDIF()

#########################
# Suppress auto-linking #
#########################

IF(MSVC_IDE)
  ADD_DEFINITIONS(-DUSING_CMAKE=1)
ENDIF()

# Pose format
# ADD_DEFINITIONS(-DVCU)

# Verbose debug prints
# ADD_DEFINITIONS(-DDEBUG)

# Instrumentation
# ADD_DEFINITIONS(-DINSTRUMENTATION)

#message(STATUS "main CUDA Include Directory: ${CUDA_INCLUDE_DIRS}")
#message(STATUS "main CUDA Library Directory: ${CUDA_LIBRARY_DIRS}")


######################
# Add subdirectories #
######################

#ADD_SUBDIRECTORY(Apps)
#ADD_SUBDIRECTORY(FernRelocLib)
#ADD_SUBDIRECTORY(InputSource)
#ADD_SUBDIRECTORY(ITMLib)
#ADD_SUBDIRECTORY(MiniSlamGraphLib)
#ADD_SUBDIRECTORY(ORUtils)
add_library(plugin SHARED plugin.cpp)
#TARGET_LINK_LIBRARIES(plugin ${DRACO_LIB} CLIEngine InputSource ITMLib MiniSlamGraphLib ORUtils FernRelocLib)
TARGET_LINK_LIBRARIES(plugin ${DRACO_LIB})
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
target_link_libraries(plugin ${Boost_LIBRARIES} ${OpenCV_LIBS})
