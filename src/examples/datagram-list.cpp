/*
 *  Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */
#ifndef MAIN_CPP
#define MAIN_CPP

#include "../datagrams/DatagramParserFactory.hpp"
#include <iostream>
#include <string>

/**Write the information about the datagram-list*/
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

class DatagramPrinter : public DatagramEventHandler{
	public:

                /**
                 * Create a datagram printer and open all the files
                 */
		DatagramPrinter(){

		}

                /**Destroy the datagram printer and close all the files*/
		~DatagramPrinter(){

		}

                /**
                 * Write a new line with a tag at the start
                 * 
                 * tag the tag
                 */
		void processDatagramTag(int tag){
			printf("%d\n",tag);
		}
};

/**
  * declare the parser depending on argument receive
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

		parser->parse(fileName);
	}
	catch(const char * error){
		std::cerr << "Error whille parsing " << fileName << ": " << error << std::endl;
	}


	if(parser) delete parser;
}


#endif
