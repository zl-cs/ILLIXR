##########################################
# Top-level CMakeLists.txt for InfiniTAM #
##########################################

cmake_minimum_required(VERSION 2.8)

PROJECT(InfiniTAM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
#################################
# Specify the CMake module path #
#################################

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
# INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
# INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseFFmpeg.cmake)
# INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseGLUT.cmake)
# INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseLibRoyale.cmake)
# INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenGL.cmake)
# INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
# INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
# INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UsePNG.cmake)
# INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseRealSense.cmake)
# INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseRealSense2.cmake)
# INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseUVC.cmake)

SET(sources
InfiniTAM.cpp
UIEngine.cpp
)

SET(headers
UIEngine.h
)

# SOURCE_GROUP("" FILES ${sources} ${headers})
# INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDAAppTarget.cmake)


#################################
# Add additional compiler flags #
#################################

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
ENDIF()

IF(NOT MSVC_IDE)
  SET(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  SET(CMAKE_CXX_FLAGS "-fPIC -O3 -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
  #SET(CMAKE_CXX_FLAGS "-fPIC -g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
ENDIF()

#########################
# Suppress auto-linking #
#########################

IF(MSVC_IDE)
  ADD_DEFINITIONS(-DUSING_CMAKE=1)
ENDIF()

######################
# Add subdirectories #
######################

ADD_SUBDIRECTORY(Apps)
#ADD_SUBDIRECTORY(FernRelocLib)
#ADD_SUBDIRECTORY(InputSource)
ADD_SUBDIRECTORY(ITMLib)
#ADD_SUBDIRECTORY(MiniSlamGraphLib)
#ADD_SUBDIRECTORY(ORUtils)

message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
# message(${PROJECT_SOURCE_DIR})
# INCLUDE(/home/bytian/Desktop/ILLIXR/infinitam/cmake/LinkFFmpeg.cmake)
# INCLUDE(/home/bytian/Desktop/ILLIXR/infinitam/cmake/LinkGLUT.cmake)
# INCLUDE(/home/bytian/Desktop/ILLIXR/infinitam/cmake/LinkLibRoyale.cmake)
# INCLUDE(/home/bytian/Desktop/ILLIXR/infinitam/cmake/LinkOpenGL.cmake)
# INCLUDE(/home/bytian/Desktop/ILLIXR/infinitam/cmake/LinkOpenNI.cmake)
# INCLUDE(/home/bytian/Desktop/ILLIXR/infinitam/cmake/LinkPNG.cmake)
# INCLUDE(/home/bytian/Desktop/ILLIXR/infinitam/cmake/LinkRealSense.cmake)
# INCLUDE(/home/bytian/Desktop/ILLIXR/infinitam/cmake/LinkRealSense2.cmake)
# INCLUDE(/home/bytian/Desktop/ILLIXR/infinitam/cmake/LinkUVC.cmake)
message("=======================================================================")

#add_library(plugin STATIC plugin.cpp)
add_library(plugin SHARED plugin.cpp)

TARGET_LINK_LIBRARIES(plugin ITMLib)

# TARGET_LINK_LIBRARIES(plugin InputSource ITMLib MiniSlamGraphLib ORUtils FernRelocLib ${PROJECT_SOURCE_DIR}/cmake/LinkFFmpeg.cmake ${PROJECT_SOURCE_DIR}/cmake/LinkGLUT.cmake ${PROJECT_SOURCE_DIR}/cmake/LinkLibRoyale.cmake ${PROJECT_SOURCE_DIR}/cmake/LinkOpenGL.cmake ${PROJECT_SOURCE_DIR}/cmake/LinkOpenNI.cmake ${PROJECT_SOURCE_DIR}/cmake/LinkPNG.cmake ${PROJECT_SOURCE_DIR}/cmake/LinkRealSense.cmake ${PROJECT_SOURCE_DIR}/cmake/LinkRealSense2.cmake ${PROJECT_SOURCE_DIR}/cmake/LinkUVC.cmake) 

                                                           
