cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find ROS
find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

# Find ROS Messages
find_package(std_msgs REQUIRED)
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(_modelname_plugin SHARED ModelName_plugin.cc)
target_link_libraries(_modelname_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})

set(dir ${CMAKE_CURRENT_SOURCE_DIR}/lib/)
set(LIBRARY_OUTPUT_PATH ${dir} CACHE PATH "Build directory" FORCE)
