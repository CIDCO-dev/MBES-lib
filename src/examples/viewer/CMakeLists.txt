cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(EEG)
find_package(PCL 1.3 REQUIRED)
find_package(Qt5Widgets REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(georefPCLviewer src/georefPCLviewer.cpp)
target_link_libraries(georefPCLviewer ${PCL_LIBRARIES})
