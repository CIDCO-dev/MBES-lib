cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(VIEWER)
find_package(PCL 1.3 REQUIRED)
find_package(Qt5Widgets REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(viewer viewer.cpp)
target_link_libraries(viewer ${PCL_LIBRARIES})
