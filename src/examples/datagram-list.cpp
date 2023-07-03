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

#ifdef _WIN32
#pragma comment(lib, "Ws2_32.lib")
#endif
/**Writes the usage information about the datagram-list*/
void printUsage(){
	std::cerr << "\n\
	NAME\n\n\
	datagram-list - liste les datagrammes contenus dans un fichier binaire\n\n\
	SYNOPSIS\n \
	datagram-list fichier\n\n\
	DESCRIPTION\n\n \
	Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés" << std::endl;
	exit(1);
}

/*!
* \brief Datagram Printer class
*
* Extends DatagramEventHandler.
*/
class DatagramPrinter : public DatagramEventHandler{
public:

	/**
	* Creates a datagram printer and open all the files
	*/
	DatagramPrinter(){

	}

	/**Destroys the datagram printer and closes all the files*/
	~DatagramPrinter(){

	}

	/**
	* Writes a new line with a tag at the start
	*
	* @param tag The datagram tag
	*/
	void processDatagramTag(int tag){
		//also display character value for printable characters
		std::stringstream printableValue;

		if(tag >= 48 && tag <= 122){
			printableValue << " (" << (char)tag << ")";
		}

		printf("%d%s\n",tag,printableValue.str().c_str());
	}
};

/**
* Declares the parser depending on argument received
*
* @param argc number of argument
* @param argv value of the arguments
*/
int main (int argc , char ** argv ){
	DatagramParser * parser = NULL;
	DatagramPrinter  printer;

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

		parser = DatagramParserFactory::build(fileName,printer);
		parser->parse(fileName, true);
	}
	catch(Exception * error){
		std::cerr << "Error whille parsing " << fileName << ": " << error->what() << std::endl;
	}


	if(parser) delete parser;
}


#endif
