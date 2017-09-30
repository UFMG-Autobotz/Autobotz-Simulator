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

add_library(Data SHARED Data.cpp)
target_link_libraries(Data ${GAZEBO_libraries})

add_library(vt_sim_plugin SHARED VT_sim_plugin.cpp)
target_link_libraries(vt_sim_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES} Data)

set(dir ${CMAKE_CURRENT_SOURCE_DIR}/import/)
set(LIBRARY_OUTPUT_PATH ${dir} CACHE PATH "Build directory" FORCE)
