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
				//unsigned char *buffer;
				
				if( fread(buffer, header.numBytesDgm-sizeof(EMdgmHeader), 1, file) == 1){
					processDatagram(header, buffer);
				}
				
				free(buffer);
				
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

void KmallParser::processDatagram(EMdgmHeader & header, unsigned char * datagram){
		
	std::string datagramType(reinterpret_cast<char *>(header.dgmType), sizeof(header.dgmType));
	std::cerr<< datagramType << "\n";

	if(datagramType == "#IIP"){

	}
	
	else if(datagramType == "#IOP"){
		//meh
	}
	
	else if(datagramType == "#SPO"){
			
		processSPO(header, datagram);
		
	}
	else if(datagramType == "#SKM"){
		exit(1);
	}
	else if(datagramType == "#SVP"){
		processSVP(header, datagram);		
	}
	
	else if(datagramType == "#SVT"){
		exit(1);
	}
	
	else if(datagramType == "#SCL"){
		exit(1);
	}
	
	else if(datagramType == "#SDE"){
		
	}
	
	else if(datagramType == "#SHI"){
		exit(1);
	}
	
	else if(datagramType == "#MRZ"){
		processMRZ(header, datagram);
		exit(1);
	}
	
	else if(datagramType == "#MWC"){
		exit(1);
	}
	
	else if(datagramType == "#CPO"){
		//meh
	}
	
	else if(datagramType == "#CHE"){
		//meh
	}
	
	else if(datagramType == "#FCF"){
		//meh
	}
	
	else{
		throw new Exception("Unkonwn Datagram");
	}
}

void KmallParser::processSVP(EMdgmHeader & header, unsigned char * datagram){
	
	EMdgmSVP_def svp;	
	unsigned char *p;
	
	memset(&svp, 0, sizeof(EMdgmSVP_def)); // XXX not sure if essential
	
	p = (unsigned char*)&svp;
	
	memcpy(p, (char*)&header, sizeof(header));
	p+=sizeof(EMdgmHeader);
	
	memcpy(p, datagram, header.numBytesDgm-sizeof(EMdgmHeader));
	
	std::cerr << svp.numBytesCmnPart <<"\n";
	
	for(int i = 0; i < svp.numBytesCmnPart; i++){
		EMdgmSVPpoint_def svpPoint = svp.sensorData[i];
		
		//TODO
		//std::cerr<<svpPoint.depth_m<<"\n";
	}
}

void KmallParser::processSPO(EMdgmHeader & header, unsigned char * datagram){

	EMdgmSPO_def spo;	
	unsigned char *p;
	
	memset(&spo, 0, sizeof(EMdgmSPO_def)); // XXX not sure if essential
	
	p = (unsigned char*)&spo;
	
	memcpy(p, (char*)&header, sizeof(header));
	p+=sizeof(EMdgmHeader);
	
	memcpy(p, datagram, header.numBytesDgm-sizeof(EMdgmHeader));
	
	//TODO
	
	/*
	std::cerr<<spo.cmnPart.numBytesCmnPart <<"\n";
	
	std::cerr<<"timeFromSensor_sec: " << spo.sensorData.timeFromSensor_sec <<"\n";
	std::cerr<<"posFixQuality_m: " << spo.sensorData.posFixQuality_m <<"\n";
	std::cerr<<"correctedLat_deg: " << spo.sensorData.correctedLat_deg <<"\n";
	std::cerr<<"correctedLong_deg: " << spo.sensorData.correctedLong_deg <<"\n";
	
	std::cerr<< spo.header.numBytesDgm <<"\n";
	std::cerr<< header.numBytesDgm <<"\n";
	*/

}

void KmallParser::processMRZ(EMdgmHeader & header, unsigned char * datagram){

	EMdgmMRZ_def mrz;	
	unsigned char *p;
	
	memset(&mrz, 0, sizeof(EMdgmMRZ_def)); // XXX not sure if essential
	
	p = (unsigned char*)&mrz;
	
	memcpy(p, (char*)&header, sizeof(header));
	p+=sizeof(EMdgmHeader);
	
	memcpy(p, datagram, header.numBytesDgm-sizeof(EMdgmHeader));
	
	//TODO

}

#endif
