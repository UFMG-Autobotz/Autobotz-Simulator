cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(roscpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(_debug_link_data_parser SHARED DebugLinkDataParser.cc)
target_link_libraries(_debug_link_data_parser ${GAZEBO_libraries})

add_library(_debug_link_plugin SHARED DebugLinkPlugin.cc)
target_link_libraries(_debug_link_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES} _debug_link_data_parser)
add_dependencies(_debug_link_plugin _debug_link_data_parser)

# Choose where the plugin will be built
set(dir ${CMAKE_CURRENT_SOURCE_DIR}/lib/)
set(LIBRARY_OUTPUT_PATH ${dir} CACHE PATH "Build directory" FORCE)
