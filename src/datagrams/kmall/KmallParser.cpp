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
			
			if(elementsRead == 1){ //
				/*
				std::cerr<<"header.numBytesDgm: " << (unsigned int)header.numBytesDgm <<"\n";
				std::cerr<<"header.dgmType: " << header.dgmType <<"\n";
				std::cerr<<"header.dgmVersion: " << (unsigned int)header.dgmVersion <<"\n";
				std::cerr<<"header.echoSounderID: " << header.echoSounderID <<"\n";
				std::cerr<<"header.time_sec: " << header.time_sec <<"\n";
				*/
				
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
				if(!feof(file)){
					std::cerr<<"Not enough bytes to read header" << std::endl;
				}
			}
			
		} // while loop end of file
	}
	else{
		std::cerr<<"Cannot open file"<< std::endl;
	}
}

void KmallParser::processDatagram(EMdgmHeader & header, unsigned char * datagram){
		
	std::string datagramType(reinterpret_cast<char *>(header.dgmType), sizeof(header.dgmType));
	//std::cerr<< datagramType << "\n";

	if(datagramType == "#IIP"){
	}
	
	else if(datagramType == "#IOP"){
	}
	
	else if(datagramType == "#SPO"){
		// position
		processSPO(header, datagram);
	}
	
	else if(datagramType == "#SKM"){
		// attitude
		processSKM(header, datagram);
	}
	
	else if(datagramType == "#SVP"){
		processSVP(header, datagram);		
	}
	
	else if(datagramType == "#SVT"){
		processSVT(header, datagram);
	}
	
	else if(datagramType == "#SCL"){
		//processSCL(header, datagram);
	}
	
	else if(datagramType == "#SDE"){
		//processSDE(header, datagram);
	}
	
	else if(datagramType == "#SHI"){
		//processSHI(header, datagram);
	}
	
	else if(datagramType == "#MRZ"){
		// mbes ping
		processMRZ(header, datagram);
		//exit(1);
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

void KmallParser::processSVP(EMdgmHeader & header, unsigned char * datagram){
	if(header.dgmVersion == SVP_VERSION){
		
		EMdgmSVPpoint_def * sensorData = (EMdgmSVPpoint_def *) (datagram + 2*sizeof(uint16_t) + 4*sizeof(uint8_t) + sizeof(uint32_t) + 2*sizeof(double));
		
		double lat = *((double *)(datagram + 2*sizeof(uint16_t) + 4*sizeof(uint8_t) + sizeof(uint32_t)));
		double lon = *((double *)(datagram + 2*sizeof(uint16_t) + 4*sizeof(uint8_t) + sizeof(uint32_t) + sizeof(double)));
		
		SoundVelocityProfile * SVP = new SoundVelocityProfile();
		SVP->setTimestamp(TimeUtils::buildTimeStamp(header.time_sec, header.time_nanosec));
		
		if(lat != UNAVAILABLE_LATITUDE && lon != UNAVAILABLE_LONGITUDE){
			SVP->setLatitude(lat);
			SVP->setLongitude(lon);
		}
		
		for(int i = 0; i < *((uint16_t*)(datagram+sizeof(uint16_t))); i++){
			SVP->add(sensorData[i].depth_m, sensorData[i].soundVelocity_mPerSec);
		}
	}
	else{
		throw new Exception("Datagram version read != Datagram version in code");
	}
}

void KmallParser::processSPO(EMdgmHeader & header, unsigned char * datagram){
	if(header.dgmVersion == SPO_VERSION){
		
		EMdgmSPOdataBlock_def * sensorData = (EMdgmSPOdataBlock_def *)(datagram + sizeof(EMdgmScommon_def));
		
		processor.processPosition(
			TimeUtils::buildTimeStamp(header.time_sec, header.time_nanosec),
			sensorData->correctedLong_deg,
			sensorData->correctedLat_deg,
			sensorData->ellipsoidHeightReRefPoint_m
		);
	}
	else{
		throw new Exception("Datagram version read != Datagram version in code");
	}

}

void KmallParser::processMRZ(EMdgmHeader & header, unsigned char * datagram){

	if(header.dgmVersion == MRZ_VERSION){
	
		unsigned int size = *(uint16_t*)(datagram + sizeof(EMdgmMpartition_def));
		
		EMdgmMRZ_pingInfo *pingInfo = (EMdgmMRZ_pingInfo*)(datagram + sizeof(EMdgmMpartition_def) + sizeof(EMdgmMbody_def));

		//Get sector
		int nBytesTx = pingInfo->numBytesPerTxSector;
		int numTxSectors = pingInfo->numTxSectors;	
		EMdgmMRZ_txSectorInfo_def * sectors = (EMdgmMRZ_txSectorInfo_def*)(datagram + sizeof(EMdgmMpartition_def) + sizeof(EMdgmMbody_def) +
		sizeof(EMdgmMRZ_pingInfo_def));
		
		float tiltAngles[numTxSectors];
		
		for(int i = 0; i<numTxSectors; i++){
			tiltAngles[i] = sectors[i].tiltAngleReTx_deg;
		}
				
		EMdgmMRZ_rxInfo *rxInfo = (EMdgmMRZ_rxInfo*)(datagram + sizeof(EMdgmMpartition_def) + sizeof(EMdgmMbody_def) +
		sizeof(EMdgmMRZ_pingInfo_def) +  (nBytesTx * numTxSectors) );

	
		EMdgmMRZ_sounding* soundings = (EMdgmMRZ_sounding*)(datagram + sizeof(EMdgmMpartition_def) + sizeof(EMdgmMbody_def) +
		sizeof(EMdgmMRZ_pingInfo_def) +  (nBytesTx * numTxSectors) + rxInfo->numBytesRxInfo + 
		(rxInfo->numExtraDetectionClasses * sizeof(EMdgmMRZ_extraDetClassInfo)));
				
		for(uint32_t i = 0; i<rxInfo->numSoundingsMaxMain; i++){
			/*
			std::cerr<<"soundingIndex: " << (soundings+i)->soundingIndex <<"\n";
			std::cerr<<"qualityFactor: " << (soundings+i)->qualityFactor <<"\n";
			std::cerr<<"beamAngleReRx_deg: " << (soundings+i)->beamAngleReRx_deg <<"\n";
			std::cerr<<"twoWayTravelTime_sec: " << (soundings+i)->twoWayTravelTime_sec <<"\n";
			std::cerr<<"beamIncAngleAdj_deg: " << (soundings+i)->beamIncAngleAdj_deg <<"\n";
			std::cerr<<"txSectorNumb: " << (int)((soundings+i)->txSectorNumb) << "\n\n";
			*/
			
			processor.processPing(
				TimeUtils::buildTimeStamp(header.time_sec, header.time_nanosec),
				( long )( (soundings+i)->soundingIndex),
				(double)( (soundings+i)->beamAngleReRx_deg),
				(double)( tiltAngles[(soundings+i)->txSectorNumb]),
				(double)( (soundings+i)->twoWayTravelTime_sec),
				static_cast<uint32_t>( ( (soundings+i)->qualityFactor) ), 
				static_cast<uint32_t>((soundings+i)->reflectivity1_dB)
			);
		}
	}
	else{
		throw new Exception("Datagram version read != Datagram version in code");
	}
	 
}

void KmallParser::processSKM(EMdgmHeader & header, unsigned char * datagram){
	if(header.dgmVersion == SKM_VERSION){
			
		EMdgmSKMinfo_def * infoPart = (EMdgmSKMinfo_def*)datagram;
		
		if(!(infoPart->sensorStatus & 0x10)){
			
			int32_t nbSamples = (header.numBytesDgm-sizeof(EMdgmHeader) - sizeof(EMdgmSKMinfo_def)) / sizeof(EMdgmSKMsample_def);
			
			EMdgmSKMsample_def * sample = (EMdgmSKMsample_def *)(datagram + sizeof(EMdgmSKMinfo_def));
			
			for(int i = 0; i < nbSamples; i++){
				processor.processAttitude(
					TimeUtils::buildTimeStamp(sample[i].KMdefault.time_sec, sample[i].KMdefault.time_nanosec), //XXX
					sample[i].KMdefault.heading_deg,
					sample[i].KMdefault.pitch_deg,
					sample[i].KMdefault.roll_deg				
					);
			}
		}
	}
	else{
		throw new Exception("Datagram version read != Datagram version in code");
	}
}

void KmallParser::processSCL(EMdgmHeader & header, unsigned char * datagram){
	if(header.dgmVersion == SCL_VERSION){
		EMdgmSCL_def scl;	
		unsigned char *p;
		
		memset(&scl, 0, sizeof(EMdgmSCL_def)); // XXX not sure if essential
		
		p = (unsigned char*)&scl;
		
		memcpy(p, (char*)&header, sizeof(header));
		p+=sizeof(EMdgmHeader);
		
		memcpy(p, datagram, header.numBytesDgm-sizeof(EMdgmHeader));
		
		/*
		std::cerr<<scl.cmnPart.numBytesCmnPart<<"\n";
		std::cerr<<scl.sensData.offset_sec<<"\n";
		*/
	}
	else{
		throw new Exception("Datagram version read != Datagram version in code");
	}
}


void KmallParser::processSVT(EMdgmHeader & header, unsigned char * datagram){
	if(header.dgmVersion == SVT_VERSION){
		
		EMdgmSVTsample_def * sensorData = (EMdgmSVTsample_def *)(datagram + sizeof(EMdgmSVTinfo_def));
		
		//std::cerr<<"sensor data: " << sensorData->soundVelocity_mPerSec << std::endl;
		
		processor.processSwathStart((double)(sensorData->soundVelocity_mPerSec));
		
	}
	else{
		throw new Exception("Datagram version read != Datagram version in code");
	}
}

void KmallParser::processSDE(EMdgmHeader & header, unsigned char * datagram){
	
	if(header.dgmVersion == SDE_VERSION){
		EMdgmSDE_def sde;	
		unsigned char *p;
		
		memset(&sde, 0, sizeof(EMdgmSDE_def)); // XXX not sure if essential
		
		p = (unsigned char*)&sde;
		
		memcpy(p, (char*)&header, sizeof(header));
		p+=sizeof(EMdgmHeader);
		
		memcpy(p, datagram, header.numBytesDgm-sizeof(EMdgmHeader));
	}
	else{
		throw new Exception("Datagram version read != Datagram version in code");
	}
}

void KmallParser::processSHI(EMdgmHeader & header, unsigned char * datagram){
	if(header.dgmVersion == SHI_VERSION){
		EMdgmSHI_def shi;	
		unsigned char *p;
		
		memset(&shi, 0, sizeof(EMdgmSHI_def)); // XXX not sure if essential
		
		p = (unsigned char*)&shi;
		
		memcpy(p, (char*)&header, sizeof(header));
		p+=sizeof(EMdgmHeader);
		
		memcpy(p, datagram, header.numBytesDgm-sizeof(EMdgmHeader));
	}
	else{
		throw new Exception("Datagram version read != Datagram version in code");
	}
}


#endif
