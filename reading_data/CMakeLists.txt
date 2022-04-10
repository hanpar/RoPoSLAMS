cmake_minimum_required(VERSION 3.10)

project(read_data)

add_executable(read_data data_read.cpp)
#set_target_properties(Task1 PROPERTIES CXX_EXTENSIONS OFF)
find_package(GTSAM REQUIRED)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${GTSAM_INCLUDE_DIR})
target_link_libraries(read_data gtsam ${PYTHON_LIBRARIES})
