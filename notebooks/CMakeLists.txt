cmake_minimum_required(VERSION 3.10)
project(lane_clustering)

# Find required packages
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# Add executable
add_executable(lane_cluster lane_cluster.cpp)

# Link libraries
target_link_libraries(lane_cluster
    ${PCL_LIBRARIES}
    cnpy
)

# Add compiler flags
add_definitions(${PCL_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PCL_CXX_FLAGS}") 