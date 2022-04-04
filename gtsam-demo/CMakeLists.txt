project(OdometryExample)
cmake_minimum_required(VERSION 2.8)

find_package(GTSAM REQUIRED)

include_directories(${GTSAM_INCLUDE_DIR})

add_executable(OdometryExample OdometryExample.cpp)
add_executable(LocalizationExample LocalizationExample.cpp)
add_executable(PlanarSLAMExample PlanarSLAMExample.cpp)

target_link_libraries(OdometryExample gtsam)
target_link_libraries(LocalizationExample gtsam)
target_link_libraries(PlanarSLAMExample gtsam)