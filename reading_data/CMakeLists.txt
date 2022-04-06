project(data_read)
cmake_minimum_required(VERSION 2.8)

find_package(GTSAM REQUIRED)

include_directories(${GTSAM_INCLUDE_DIR})

add_executable(data_read data_read.cpp)
target_link_libraries(data_read gtsam)