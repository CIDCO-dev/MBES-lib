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

	//std::cerr<< header.dgmType <<"\n";
	//std::cerr<< header.dgmType[5] <<"\n"; 
	std::cerr<< datagramType << "\n";

	if(datagramType == "#IIP"){
		/*
		unsigned char * buffer = (unsigned char*)malloc(sizeof(EMdgmIIP_def));
		memcpy(buffer, (char*)&header, sizeof(EMdgmHeader));
		buffer += sizeof(EMdgmHeader);
		
		std::cerr<<"\n" << sizeof(EMdgmHeader) <<"\n";
		std::cerr<< header.numBytesDgm <<"\n";
		std::cerr<<sizeof(EMdgmIIP_def) <<"\n\n";
		
		memcpy(buffer, (char*)&datagram, header.numBytesDgm-sizeof(EMdgmHeader)+4);
		
		//memcpy(buffer, (char*)&datagram, sizeof(EMdgmIIP_def));
		
		//EMdgmIIP_def *iip = (EMdgmIIP_def*)buffer;
		
		free(buffer);
		*/
	}
	
	else if(datagramType == "#IOP"){
		//meh
	}
	
	else if(datagramType == "#SPO"){
			
		processSPO(header, datagram);
		exit(1);
		
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
	
	unsigned int nbByte = 0;
	EMdgmSVPpoint *svp = new EMdgmSVPpoint;
	while(nbByte < header.numBytesDgm){
		memcpy(svp, (char*)&datagram, sizeof(EMdgmSVPpoint));
		
		// TODO do something with the datagram
		//std::cerr<<svp->depth_m<<"\n";
		
		nbByte += sizeof(EMdgmSVPpoint);
		datagram += sizeof(EMdgmSVPpoint);
	}
	
	delete svp;
}

void KmallParser::processSPO(EMdgmHeader & header, unsigned char * datagram){
	
	EMdgmScommon *common = new EMdgmScommon;
	memcpy(common, (uint32_t*)&datagram, sizeof(EMdgmScommon));
	
	std::cerr<<"common->numBytesInfoPart: " << common->numBytesCmnPart <<"\n";
	std::cerr<<"common->sensorStatus: " << common->sensorStatus <<"\n";
	std::cerr<<"common->padding: " << common->padding <<"\n";
	
	datagram += sizeof(EMdgmScommon);
	EMdgmSPOdataBlock_def *data = new EMdgmSPOdataBlock_def;
	memcpy(data, (char*)&datagram, header.numBytesDgm - sizeof(EMdgmScommon));
	
	std::cerr<<"timeFromSensor_sec: " << data->timeFromSensor_sec <<"\n";
	std::cerr<<"posFixQuality_m: " << data->posFixQuality_m <<"\n";
	std::cerr<<"correctedLat_deg: " << data->correctedLat_deg <<"\n";
	std::cerr<<"correctedLong_deg: " << data->correctedLong_deg <<"\n";

}
#endif
