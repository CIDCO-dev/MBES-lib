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
				
				std::cerr<<"header.numBytesDgm: " << (unsigned int)header.numBytesDgm <<"\n";
				std::cerr<<"header.dgmType: " << header.dgmType <<"\n";
				std::cerr<<"header.dgmVersion: " << (unsigned int)header.dgmVersion <<"\n";
				std::cerr<<"header.echoSounderID: " << header.echoSounderID <<"\n";
				std::cerr<<"header.time_sec: " << header.time_sec <<"\n";
				
				
				if(header.dgmType[0] != '#'){
					throw new Exception("Parsing error : Datagram type should start with : # ");
				}
				
				
				
				unsigned char * buffer = (unsigned char*)malloc(header.numBytesDgm - sizeof(EMdgmHeader));
				if( fread(buffer, header.numBytesDgm-sizeof(EMdgmHeader), 1, file) == 1){
					processDatagram(header, buffer);
				}
			}
			else{
				std::cerr<<"Not enoug bytes to read header" << std::endl;
			}
		} // while loop end of file
	}
	else{
		std::cerr<<"Cannot open file"<< std::endl;
	}
}


void KmallParser::processDatagramIIP(EMdgmHeader & header, EMdgmIIP_def & iip){
	
	std::cerr<<(uint16_t)iip.numBytesCmnPart<<"\n";
	std::cerr<<(uint16_t)iip.info<<"\n";
	std::cerr<<(uint16_t)iip.status<<"\n";
	std::cerr<<(uint8_t)iip.install_txt<<"\n";

}


void KmallParser::processDatagram(EMdgmHeader & header, unsigned char * datagram){

	std::string datagramType = reinterpret_cast<char *>(header.dgmType);
	
	std::cerr<< datagramType << "\n";

	if(datagramType == "#IIP"){
			
	}
	else if(datagramType == "#IOP"){
		
	}
	else if(datagramType == "#SPO"){
		
	}
	else if(datagramType == "#SKM"){
		
	}
	else if(datagramType == "#SVP"){
		
	}
	else if(datagramType == "#SVT"){
		
	}
	else if(datagramType == "#SCL"){
		
	}
	else if(datagramType == "#SDE"){
		
	}
	else if(datagramType == "#SHI"){
		
	}
	else if(datagramType == "#MRZ"){

	}
	else if(datagramType == "#MWC"){
		
	}
	else if(datagramType == "#CPO"){
		
	}
	else if(datagramType == "#CHE"){
		
	}
	else if(datagramType == "#FCF"){
		
	}
	else{
		throw new Exception("Unkonwn Datagram");
	}
}
#endif
