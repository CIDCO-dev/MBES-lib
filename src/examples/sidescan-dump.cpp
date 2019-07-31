/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/*
* \author Guillaume Labbe-Morissette
*/

#ifndef MAIN_CPP
#define MAIN_CPP

#include "../datagrams/DatagramParserFactory.hpp"
#include <iostream>
#include <string>

/**Writes the usage information about the datagram-list*/
void printUsage(){
	std::cerr << "\n\
	NAME\n\n\
	sidescan-dump - Dumps sidescan data to an image file\n\n\
	SYNOPSIS\n \
	sidescan-dump file\n\n\
	DESCRIPTION\n\n \
	Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), All rights reserved" << std::endl;
	exit(1);
}

/*!
* \brief Datagram Printer class
*
* Extends DatagramEventHandler.
*/
class SidescanDataDumper : public DatagramEventHandler{
public:

	/**
	* Creates a SidescanDataDumper
	*/
	SidescanDataDumper(){

	}

	/**Destroys the SidescanDataDumper*/
	~SidescanDataDumper(){

	}

        void processSidescanData(std::vector<double> & data){
            std::cout << data.size() << " samples" << std::endl;
        }
};

/**
* Declares the parser depending on argument received
*
* @param argc number of argument
* @param argv value of the arguments
*/
int main (int argc , char ** argv ){
	DatagramParser *    parser      = NULL;
	SidescanDataDumper  sidescan;

	#ifdef __GNU__
	setenv("TZ", "UTC", 1);
	#endif
	#ifdef _WIN32
	putenv("TZ");
	#endif

	if(argc != 2){
		printUsage();
	}

	std::string fileName(argv[1]);

	try{
		std::cerr << "Decoding " << fileName << std::endl;

		parser = DatagramParserFactory::build(fileName,sidescan);

		parser->parse(fileName);
	}
	catch(std::exception * e){
		std::cerr << "Error while parsing " << fileName << ": " << e->what() << std::endl;
	}


	if(parser) delete parser;
}


#endif
