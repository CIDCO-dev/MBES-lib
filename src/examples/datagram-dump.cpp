/*
 *  Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */
#ifndef MAIN_CPP
#define MAIN_CPP

#include <getopt.h>

#include "../datagrams/kongsberg/KongsbergParser.hpp"
#include "../datagrams/xtf/XtfParser.hpp"
#include <iostream>
#include <string>
#include "../utils/StringUtils.hpp"

void printUsage(){
	std::cerr << "\n\
  NAME\n\n\
     datagram-dump - lit un fichier multibeam et le transforme en format texte (ASCII)\n\n\
  SYNOPSIS\n \
	   datagram-dump fichier\n\n\
  DESCRIPTION\n\n \
  Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés" << std::endl;
	exit(1);
}



int main (int argc , char ** argv ){

	if(argc != 2){
		printUsage();
	}

	std::string fileName(argv[1]);

	try{
		std::cerr << "Decoding " << fileName << std::endl;
		MbesParser *  parser;

		if(ends_with(fileName.c_str(),".all")){
			parser = new KongsbergParser();
		}
		else if(ends_with(fileName.c_str(),".xtf")){
			parser = new XtfParser();
		}
		else if(ends_with(fileName.c_str(),".sk7")){
			//TODO
		}
		else{
			throw "Unknown extension";
		}

		parser->parse(fileName);
	}
	catch(const char * error){
		std::cerr << "Error whille parsing " << fileName << ": " << error << std::endl;
	}
}


#endif
