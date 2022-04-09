project(2Dpose_optimization)
cmake_minimum_required(VERSION 2.8)

find_package(GTSAM REQUIRED)

include_directories(${GTSAM_INCLUDE_DIR})

add_executable(2Dpose_optimization 2Dpose_optimization.cpp)
add_executable(pose_optimization pose_optimization.cpp)

target_link_libraries(2Dpose_optimization gtsam)

target_link_libraries(pose_optimization gtsam)