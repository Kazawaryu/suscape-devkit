cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(sus_vis)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加对json的支持
include_directories(/usr/local/include)
link_directories(/usr/local/lib)


add_executable (sus_vis suscape_visualize.cpp)
target_link_libraries (sus_vis ${PCL_LIBRARIES} jsoncpp)
