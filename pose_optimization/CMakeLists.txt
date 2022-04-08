project(pose_optimization)
cmake_minimum_required(VERSION 2.8)

find_package(GTSAM REQUIRED)

include_directories(${GTSAM_INCLUDE_DIR})

add_executable(pose_optimization pose_optimization.cpp)
add_executable(pose_optimization_new pose_optimization_new.cpp)

target_link_libraries(pose_optimization gtsam)
target_link_libraries(pose_optimization_new gtsam)