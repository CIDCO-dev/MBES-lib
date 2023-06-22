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
		//processSVT(header, datagram);
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
		EMdgmSVP_def svp;	
		unsigned char *p;
		
		memset(&svp, 0, sizeof(EMdgmSVP_def)); // XXX not sure if essential
		
		p = (unsigned char*)&svp;
		
		memcpy(p, (char*)&header, sizeof(header));
		p+=sizeof(EMdgmHeader);
		
		memcpy(p, datagram, header.numBytesDgm-sizeof(EMdgmHeader));
		
		//std::cerr << svp.numBytesCmnPart <<"\n";
		
		SoundVelocityProfile * SVP = new SoundVelocityProfile();
		SVP->setTimestamp(TimeUtils::buildTimeStamp(svp.header.time_sec, svp.header.time_nanosec));
		
		for(int i = 0; i < svp.numBytesCmnPart; i++){
			SVP->add(svp.sensorData[i].depth_m, svp.sensorData[i].soundVelocity_mPerSec);
		}
	}
	else{
		throw new Exception("Datagram version read != Datagram version in code");
	}
}

void KmallParser::processSPO(EMdgmHeader & header, unsigned char * datagram){
	if(header.dgmVersion == SPO_VERSION){
		EMdgmSPO_def spo;	
		unsigned char *p;
		
		memset(&spo, 0, sizeof(EMdgmSPO_def)); // XXX not sure if essential
		
		p = (unsigned char*)&spo;
		
		memcpy(p, (char*)&header, sizeof(header));
		p+=sizeof(EMdgmHeader);
		
		memcpy(p, datagram, header.numBytesDgm-sizeof(EMdgmHeader));
		
		processor.processPosition(
			TimeUtils::buildTimeStamp(spo.header.time_sec, spo.header.time_nanosec),
			spo.sensorData.correctedLong_deg,
			spo.sensorData.correctedLat_deg,
			spo.sensorData.ellipsoidHeightReRefPoint_m
		);
		
		
		/*
		std::cerr<< TimeUtils::buildTimeStamp(spo.header.time_sec, spo.header.time_nanosec) <<"\n";
		std::cerr<<spo.cmnPart.numBytesCmnPart <<"\n";
		std::cerr<<"header.time_sec: " << spo.header.time_sec <<"\n";
		std::cerr<<"timeFromSensor_sec: " << spo.sensorData.timeFromSensor_sec <<"\n";
		std::cerr<<"posFixQuality_m: " << spo.sensorData.posFixQuality_m <<"\n";
		std::cerr<<"correctedLat_deg: " << spo.sensorData.correctedLat_deg <<"\n";
		std::cerr<<"correctedLong_deg: " << spo.sensorData.correctedLong_deg <<"\n";
		std::cerr<<"ellipsoidHeightReRefPoint_m: " << spo.sensorData.ellipsoidHeightReRefPoint_m <<"\n";
		
		std::cerr<< spo.header.numBytesDgm <<"\n";
		std::cerr<< header.numBytesDgm <<"\n";
		*/
	}
	else{
		throw new Exception("Datagram version read != Datagram version in code");
	}

}

void KmallParser::processMRZ(EMdgmHeader & header, unsigned char * datagram){

	if(header.dgmVersion == MRZ_VERSION){
		
		EMdgmMRZ_def mrz;	
		unsigned char *p;
		
		memset(&mrz, 0, sizeof(EMdgmMRZ_def)); // XXX not sure if essential
		
		p = (unsigned char*)&mrz;
		
		memcpy(p, (char*)&header, sizeof(header));
		p+=sizeof(EMdgmHeader);
		
		memcpy(p, datagram, header.numBytesDgm-sizeof(EMdgmHeader));
		
		//processPing(uint64_t microEpoch,long id, double beamAngle,double tiltAngle,double twoWayTravelTime,uint32_t quality,int32_t intensity)
		
		/*
		std::cerr<< "mrz.cmnPart.numBytesCmnPart: " << mrz.cmnPart.numBytesCmnPart <<"\n";
		std::cerr<< "mrz.cmnPart.pingCnt: " << mrz.cmnPart.pingCnt <<"\n";
		std::cerr<< "mrz.header.numBytesDgm: " << mrz.cmnPart.pingCnt <<"\n";
		*/
		
		p = (unsigned char*)(&(mrz.partition.dgmNum));
		
		int sizeCommon = mrz.cmnPart.numBytesCmnPart;
		
		//skip partition
		p+=2; // i guess thats for bytes alligment
		p+=sizeCommon;
		
		EMdgmMRZ_pingInfo *pingInfo = (EMdgmMRZ_pingInfo*)p;
		
		//std::cerr<< pingInfo->numBytesInfoData <<"\n";
		
		// skip to end of ping info
		p+=pingInfo->numBytesInfoData;
		
		//Get sector
		int nBytesTx = pingInfo->numBytesPerTxSector;
		int numTxSectors = mrz.pingInfo.numTxSectors;
		
		float tiltAngles[numTxSectors];
		
		for(int i = 0; i<numTxSectors; i++){
			//EMdgmMRZ_txSectorInfo_def sector = mrz.sectorInfo[i];
			//std::cerr<<"tiltAngleReTx_deg: " << sector.tiltAngleReTx_deg<<"\n";
			tiltAngles[i] = mrz.sectorInfo[i].tiltAngleReTx_deg;
		}
		
		
		p += (nBytesTx * numTxSectors);
		
		//std::cerr<<"numTxSectors: " << numTxSectors <<"\n";
		
		EMdgmMRZ_rxInfo *rxInfo = (EMdgmMRZ_rxInfo*)p;
		//std::cerr<<"numSoundingsMaxMain: " << rxInfo->numSoundingsMaxMain << "\n";
		uint32_t nbSoundings = rxInfo->numSoundingsMaxMain;
		

		int nSizeRXInfo = rxInfo->numBytesRxInfo;
		p += nSizeRXInfo;
		int numExtraDet = rxInfo->numExtraDetectionClasses;
		p += numExtraDet * sizeof(EMdgmMRZ_extraDetClassInfo);
		EMdgmMRZ_sounding* soundings = (EMdgmMRZ_sounding*)p;
		
		
		for(uint32_t i = 0; i<nbSoundings; i++){
			/*
			std::cerr<<"soundingIndex: " << (soundings+i)->soundingIndex <<"\n";
			std::cerr<<"qualityFactor: " << (soundings+i)->qualityFactor <<"\n";
			std::cerr<<"beamAngleReRx_deg: " << (soundings+i)->beamAngleReRx_deg <<"\n";
			std::cerr<<"twoWayTravelTime_sec: " << (soundings+i)->twoWayTravelTime_sec <<"\n";
			std::cerr<<"beamIncAngleAdj_deg: " << (soundings+i)->beamIncAngleAdj_deg <<"\n";
			std::cerr<<"txSectorNumb: " << (int)((soundings+i)->txSectorNumb) << "\n\n";
			*/
			
			processor.processPing(
				TimeUtils::buildTimeStamp(mrz.header.time_sec, mrz.header.time_nanosec),
				( long )( (soundings+i)->soundingIndex),
				(double)( (soundings+i)->beamAngleReRx_deg),
				(double)( tiltAngles[(soundings+i)->txSectorNumb]),
				(double)( (soundings+i)->twoWayTravelTime_sec),
				static_cast<uint32_t>( ( (soundings+i)->qualityFactor) * 100 ), //XXX
				static_cast<uint32_t>(66.6) //XXX
			);
			//XXX
			/*
			possible intensity : (soundings+i)->receiverSensitivityApplied_dB
							(soundings+i)->rangeFactor
				
				
				beamIncAngleAdj_deg ???
			*/
		}
	}
	else{
		throw new Exception("Datagram version read != Datagram version in code");
	}
	 
}

void KmallParser::processSKM(EMdgmHeader & header, unsigned char * datagram){
	if(header.dgmVersion == SKM_VERSION){
		EMdgmSKM_def skm;	
		unsigned char *p;
		
		memset(&skm, 0, sizeof(EMdgmSKM_def)); // XXX not sure if essential
		
		p = (unsigned char*)&skm;
		
		memcpy(p, (char*)&header, sizeof(header));
		p+=sizeof(EMdgmHeader);
		
		memcpy(p, datagram, header.numBytesDgm-sizeof(EMdgmHeader));
		
		
		uint8_t status = skm.infoPart.sensorStatus;
		uint8_t mask = 0x10;
		status = status & mask;
		
		//std::cerr<< (int)(status) << "\n"; 
		
		if(status == 16){
			//invalid data
		}
		else{
			
			int32_t nbSamples = (header.numBytesDgm-sizeof(EMdgmHeader) - sizeof(EMdgmSKMinfo_def)) / sizeof(EMdgmSKMsample_def);
			
			for(int i = 0; i < nbSamples; i++){
				processor.processAttitude(
					TimeUtils::buildTimeStamp(skm.sample[i].KMdefault.time_sec, skm.sample[i].KMdefault.time_nanosec),
					skm.sample[i].KMdefault.heading_deg,
					skm.sample[i].KMdefault.pitch_deg,
					skm.sample[i].KMdefault.roll_deg
				);
				
				/*
				std::cerr<<"skm.sample[i].KMdefault.heading_deg: " << skm.sample[i].KMdefault.roll_deg <<"\n";
				std::cerr<<"skm.sample[i].KMdefault.time_sec: " << (uint32_t)skm.sample[i].KMdefault.time_sec <<"\n";
				*/	
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
		EMdgmSVT_def svt;	
		unsigned char *p;
		
		memset(&svt, 0, sizeof(EMdgmSVT_def)); // XXX not sure if essential
		
		p = (unsigned char*)&svt;
		
		memcpy(p, (char*)&header, sizeof(header));
		p+=sizeof(EMdgmHeader);
		
		memcpy(p, datagram, header.numBytesDgm-sizeof(EMdgmHeader));
		/*
		std::cerr<<svt.infoPart.numBytesCmnPart<<"\n";
		std::cerr<<svt.sensorData[0].time_sec<<"\n";
		*/
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
