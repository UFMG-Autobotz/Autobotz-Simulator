cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin

add_library(data SHARED Data.cpp)
target_link_libraries(data ${GAZEBO_libraries})

add_library(pid_control_plugin SHARED PIDControlPlugin.cpp)
target_link_libraries(pid_control_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES} data)

set(dir ${CMAKE_CURRENT_SOURCE_DIR}/import/)
set(LIBRARY_OUTPUT_PATH ${dir} CACHE PATH "Build directory" FORCE)
