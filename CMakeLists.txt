cmake_minimum_required(VERSION "3.7.1")

project("MBES-lib")

message(STATUS "Eigen3 include dir: ${Eigen3_DIR}")

find_package (Eigen3 3.3 NO_MODULE)
 
if (TARGET Eigen3::Eigen)
	set(SOURCES src/getopt.c src/datagrams/DatagramParser.cpp src/datagrams/DatagramParserFactory.cpp src/datagrams/s7k/S7kParser.cpp src/datagrams/kongsberg/KongsbergParser.cpp src/datagrams/xtf/XtfParser.cpp src/utils/NmeaUtils.cpp src/utils/StringUtils.cpp src/sidescan/SidescanPing.cpp)
	
	#georeference
	add_executable(georeference src/examples/georeference.cpp ${SOURCES})
	target_link_libraries (georeference Eigen3::Eigen)
	
	#datagram-dump
	add_executable(datagram-dump src/examples/datagram-dump.cpp ${SOURCES})
	target_link_libraries (datagram-dump Eigen3::Eigen)
	
	#cidco-decoder
	add_executable(cidco-decoder src/examples/cidco-decoder.cpp ${SOURCES})
	target_link_libraries (cidco-decoder Eigen3::Eigen)
	
	#datagram-list
	add_executable("datagram-list" src/examples/datagram-list.cpp ${SOURCES})
	target_link_libraries ("datagram-list" Eigen3::Eigen)
	
	#data-cleaning
	add_executable(data-cleaning src/examples/data-cleaning.cpp ${SOURCES})
	target_link_libraries (data-cleaning Eigen3::Eigen)
	

	set (CMAKE_CURRENT_BINARY_DIR "bin")

	set_target_properties(georeference datagram-dump cidco-decoder "datagram-list" data-cleaning PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
	set_target_properties(georeference datagram-dump cidco-decoder "datagram-list" data-cleaning PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR} )
	set_target_properties(georeference datagram-dump cidco-decoder "datagram-list" data-cleaning PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR} )
endif (TARGET Eigen3::Eigen)

