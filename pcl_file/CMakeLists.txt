cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(sus_vis)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(PCL 1.2 REQUIRED)

# pcl
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# jsoncpp
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable (sus_vis online.cpp)
target_link_libraries (sus_vis ${PCL_LIBRARIES} jsoncpp)

add_library(sus_vis_lib SHARED online.cpp)
target_link_libraries (sus_vis_lib ${PCL_LIBRARIES} jsoncpp)
