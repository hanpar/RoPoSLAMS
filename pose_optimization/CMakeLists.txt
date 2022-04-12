cmake_minimum_required(VERSION 3.10)

project(2Dpose_optimization)

#add_executable(3Dpose_optimization 3Dpose_optimization.cpp)
add_executable(2Dpose_optimization_ALOAM 2Dpose_optimization.cpp)
add_executable(2Dpose_optimization_FLOAM 2Dpose_optimization_notsync.cpp)
#add_executable(plot plot.cpp)

#set_target_properties(Task1 PROPERTIES CXX_EXTENSIONS OFF)
find_package(GTSAM REQUIRED)
find_package(PythonLibs REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${GTSAM_INCLUDE_DIR})

#target_link_libraries(3Dpose_optimization gtsam ${PYTHON_LIBRARIES})
target_link_libraries(2Dpose_optimization_ALOAM gtsam ${PYTHON_LIBRARIES})
target_link_libraries(2Dpose_optimization_FLOAM gtsam ${PYTHON_LIBRARIES})
#target_link_libraries(plot gtsam ${PYTHON_LIBRARIES})
