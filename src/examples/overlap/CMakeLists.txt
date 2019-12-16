cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)

project(overlap)

find_package(PCL 1.2 REQUIRED)
find_package(Qt5Widgets REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (overlap overlap.cpp      ../../datagrams/DatagramParserFactory.cpp ../../datagrams/DatagramParser.cpp ../../datagrams/xtf/XtfParser.cpp ../../datagrams/s7k/S7kParser.cpp ../../datagrams/kongsberg/KongsbergParser.cpp ../../utils/NmeaUtils.cpp ../../utils/StringUtils.cpp    ../../sidescan/SidescanPing.cpp     )
target_link_libraries (overlap ${PCL_LIBRARIES})

