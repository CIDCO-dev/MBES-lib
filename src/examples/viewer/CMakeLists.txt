cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)

project(VIEWER)
find_package(PCL 1.3 REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package (Eigen3 3.3 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(viewer viewer.cpp ../../src/svp/SoundVelocityProfile.cpp ../../src/datagrams/DatagramParserFactory.cpp ../../src/datagrams/DatagramParser.cpp ../../src/datagrams/xtf/XtfParser.cpp ../../src/datagrams/s7k/S7kParser.cpp ../../src/datagrams/kongsberg/KongsbergParser.cpp ../../src/utils/NmeaUtils.cpp ../../src/utils/StringUtils.cpp)

target_link_libraries(viewer ${PCL_LIBRARIES})
