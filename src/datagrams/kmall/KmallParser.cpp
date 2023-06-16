/*
* Copyright 2023 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef KMALL_CPP
#define KMALL_CPP

#include "KmallParser.hpp"


KmallParser::KmallParser(DatagramEventHandler & processor):DatagramParser(processor){

}

KmallParser::~KmallParser(){

}
void KmallParser::parse(std::string & filename, bool ignoreChecksum){
	
	std::cerr<<"kmall parser \n";
	
	FILE * file = fopen(filename.c_str(),"rb");

	if(file){
		while(!feof(file)){
			EMdgmHeader header;
			int elementsRead = fread (&header,sizeof(EMdgmHeader),1,file);
			
			if(elementsRead == 1){
				std::cerr<<"header.numBytesDgm: " << header.numBytesDgm <<"\n";
				std::cerr<<"header.dgmType: " << header.dgmType <<"\n";
				std::cerr<<"header.dgmVersion: " << header.dgmVersion <<"\n";
				std::cerr<<"header.echoSounderID: " << header.echoSounderID <<"\n";
				std::cerr<<"header.time_sec: " << header.time_sec <<"\n";
			}
			
			break;
		}
	}
}

#endif
