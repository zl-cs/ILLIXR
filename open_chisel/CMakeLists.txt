cmake_minimum_required(VERSION 2.8.8)

project(open_chisel)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -fPIC")
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)


find_package( OpenCV REQUIRED ) 
include_directories( ${OpenCV_INCLUDE_DIRS} ) 


include_directories(include)
#include_directories(illixr_port/include)

add_library(open_chisel_lib 
	src/Chunk.cpp 
	src/ChunkManager.cpp 
	src/DistVoxel.cpp  
	src/ColorVoxel.cpp
	src/geometry/AABB.cpp
	src/geometry/Plane.cpp
	src/geometry/Frustum.cpp
	src/camera/Intrinsics.cpp
	src/camera/PinholeCamera.cpp
	src/pointcloud/PointCloud.cpp
	src/ProjectionIntegrator.cpp
	src/Chisel.cpp
	src/mesh/Mesh.cpp
	src/marching_cubes/MarchingCubes.cpp
	src/io/PLY.cpp
	src/geometry/Raycast.cpp
	src/ChiselServer.cpp)

add_library(plugin SHARED plugin.cpp)
target_link_libraries(plugin open_chisel_lib)
target_link_libraries(plugin ${OpenCV_LIBS} )
#target_link_libraries(plugin PRIVATE Threads::Threads)
#target_link_libraries(open_chisel_lib)
