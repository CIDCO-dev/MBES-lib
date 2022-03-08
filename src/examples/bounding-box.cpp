/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef MAIN_CPP
#define MAIN_CPP

#include "../datagrams/DatagramParserFactory.hpp"
#include "BoundingBoxPrinter.hpp"
#include <iostream>
#include <cstdio>
#include <string>

#pragma comment(lib, "Ws2_32.lib")

/**Writes the usage information*/
void printUsage(){
	std::cerr << "\n\
	NAME\n\n\
	bounding-box - prints bounding box information for a multibeam echosounder file\n\n\
	SYNOPSIS\n \
	bounding-box file\n\n\
	DESCRIPTION\n\n \
	Copyright 2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), All rights reserved" << std::endl;
	exit(1);
}

/**
* declare the parser depending on argument receive
*
* @param argc number of argument
* @param argv value of the arguments
*/
int main (int argc , char ** argv ){
	DatagramParser * parser = NULL;
	BoundingBoxPrinter  printer;

	#ifdef __GNU__
	setenv("TZ", "UTC", 1);
	#endif
	#ifdef _WIN32
	_putenv("TZ");
	#endif

	if(argc != 2){
		printUsage();
	}

	std::string fileName(argv[1]);

	try{
		std::cerr << "Analyzing " << fileName << std::endl;

		parser = DatagramParserFactory::build(fileName,printer);

		parser->parse(fileName);

		if(
			printer.getMinimumLatitude()  != (std::numeric_limits<double>::max)()    &&
			printer.getMaximumLatitude()  != std::numeric_limits<double>::lowest() &&
			printer.getMinimumLongitude() != (std::numeric_limits<double>::max)()    &&
			printer.getMaximumLongitude() != std::numeric_limits<double>::lowest() 
		){
			printf("%.12f %.12f %.12f %.12f\n",
				printer.getMinimumLatitude(),
				printer.getMaximumLatitude(),
				printer.getMinimumLongitude(),
				printer.getMaximumLongitude()
			);
		}
		else{
			std::cerr << "No position data found\n";
			return 1;
		}

		return 0;
	}
	catch(const char * error){
		std::cerr << "Error whille parsing " << fileName << ": " << error << std::endl;
	}
	if(parser) delete parser;
}
#endif
