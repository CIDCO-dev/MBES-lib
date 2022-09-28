cmake_minimum_required(VERSION "3.7.1")

project("MBES-lib")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "Eigen3 include dir: ${Eigen3_DIR}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if (TARGET Eigen3::Eigen)
	IF(WIN32)
		ADD_DEFINITIONS(/bigobj)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
		SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
		SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /bigobj")
		SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /bigobj")
		SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /bigobj")
	ENDIF(WIN32)

	set(SOURCES src/getopt.c src/datagrams/DatagramParser.cpp src/datagrams/DatagramParserFactory.cpp src/datagrams/s7k/S7kParser.cpp src/datagrams/kongsberg/KongsbergParser.cpp src/datagrams/xtf/XtfParser.cpp src/utils/NmeaUtils.cpp src/utils/StringUtils.cpp src/sidescan/SidescanPing.cpp src/hydroblock/Hydroblock20Parser.cpp)
	
	#georeference
	add_executable(georeference src/examples/georeference.cpp ${SOURCES})
	target_link_libraries (georeference Eigen3::Eigen)

	#bounding-box
	add_executable(bounding-box src/examples/bounding-box.cpp ${SOURCES})
	target_link_libraries (bounding-box Eigen3::Eigen)

	#datagram-raytracer
	add_executable(datagram-raytracer src/examples/datagram-raytracer.cpp ${SOURCES})
	target_link_libraries (datagram-raytracer Eigen3::Eigen)

	#raytrace
	add_executable(raytrace src/examples/raytrace.cpp ${SOURCES})
	target_link_libraries (raytrace Eigen3::Eigen)

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
	
	#wgs2lgf
	add_executable(wgs2lgf src/examples/wgs2lgf.cpp ${SOURCES})
	target_link_libraries (wgs2lgf Eigen3::Eigen)
	
	set (CMAKE_CURRENT_BINARY_DIR "bin")
	
	set_target_properties(georeference bounding-box datagram-dump cidco-decoder "datagram-list" data-cleaning raytrace datagram-raytracer PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
	set_target_properties(georeference bounding-box datagram-dump cidco-decoder "datagram-list" data-cleaning raytrace datagram-raytracer PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR} )
	set_target_properties(georeference bounding-box datagram-dump cidco-decoder "datagram-list" data-cleaning raytrace datagram-raytracer PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR} )
	
	#### tests ####
	add_executable(tests test/main.cpp ${SOURCES})
	target_link_libraries (tests Eigen3::Eigen)

	set (CMAKE_CURRENT_BINARY_DIR "test")
	set_target_properties(tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
	set_target_properties(tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR})
	set_target_properties(tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR})
	
endif (TARGET Eigen3::Eigen)
