/*
* Copyright 2023 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef KMALL_CPP
#define KMALL_CPP

#include "KmallParser.hpp"

char * getMRZPingInfo(void * tgm)
{
   char *pData = NULL;

   // If this record is not empty
   if (tgm != NULL)
   {
      pEMdgmMRZ dgm = (pEMdgmMRZ)tgm;
      int sizeCommon = dgm->cmnPart.numBytesCmnPart;

      // Find rxInfo. Must take account of number of TX sectors.
      int numTxSectors = dgm->pingInfo.numTxSectors;

      //Get PingInfo
      char* pPI = (char*)(&(dgm->partition.dgmNum));
      pPI += 2;
      pPI += sizeCommon;

      pEMdgmMRZ_pingInfo pPingInfo = (pEMdgmMRZ_pingInfo)pPI;

      pData = pPI;

      // end of datagram for safety testing
      char *pSOD = (char*)dgm;
      char *pEOD = pSOD + dgm->header.numBytesDgm; // first byte past end of data

                                                   // safety check - verify pointer is valid
      if (pData >= pEOD)
      {
         pData = NULL;
      }
   }

   return pData;
}

char * getMRZSectorInfo(void * tgm)
{
   char *pData = NULL;

   // if this record is not empty
   if (tgm != NULL)
   {
      pEMdgmMRZ dgm = (pEMdgmMRZ)tgm;
      int sizeCommon = dgm->cmnPart.numBytesCmnPart;

      // Find rxInfo. Must take account of number of TX sectors.
      int numTxSectors = dgm->pingInfo.numTxSectors;

      //Get PingInfo
      char* pPI = (char*)(&(dgm->partition.dgmNum));
      pPI += 2;
      pPI += sizeCommon;

      pEMdgmMRZ_pingInfo pPingInfo = (pEMdgmMRZ_pingInfo)pPI;

      //Move to end of pingInfo
      char* pch = pPI + pPingInfo->numBytesInfoData; //At start of txsectors
      int nSkip = 0;
      pch += nSkip;

      //Get sector
      int nBytesTx = pPingInfo->numBytesPerTxSector;
      pEMdgmMRZ_txSectorInfo pSect = (pEMdgmMRZ_txSectorInfo)pch;

      pData = pch;

      // end of datagram for safety testing
      char *pSOD = (char*)dgm;
      char *pEOD = pSOD + dgm->header.numBytesDgm; // first byte past end of data

                                                   // safety check - verify pointer is valid
      if (pData >= pEOD)
      {
         pData = NULL;
      }
   }

   return pData;
}

char *getMRZRxInfo(void* tgm)
{
   char *pData = NULL;

   // if this record is not empty
   if (tgm != NULL)
   {
      pEMdgmMRZ dgm = (pEMdgmMRZ)tgm;
      int sizeCommon = dgm->cmnPart.numBytesCmnPart;

      // Find rxInfo. Must take account of number of TX sectors.
      int numTxSectors = dgm->pingInfo.numTxSectors;

      //Get PingInfo
      char* pPI = (char*)(&(dgm->partition.dgmNum));
      pPI += 2;
      pPI += sizeCommon;

      pEMdgmMRZ_pingInfo pPingInfo = (pEMdgmMRZ_pingInfo)pPI;

      //Move to end of pingInfo
      char* pch = pPI + pPingInfo->numBytesInfoData; //At start of txsectors
      int nSkip = 0;
      pch += nSkip;

      //Get sector
      int nBytesTx = pPingInfo->numBytesPerTxSector;
      pEMdgmMRZ_txSectorInfo pSect = (pEMdgmMRZ_txSectorInfo)pch;

      //Move to start of rxInfo
      pch = (char*)pSect;
      pch += (nBytesTx * numTxSectors);
      EMdgmMRZ_rxInfo* pRxInfo = (EMdgmMRZ_rxInfo*)pch;

      pData = pch;

      // end of datagram for safety testing
      char *pSOD = (char*)dgm;
      char *pEOD = pSOD + dgm->header.numBytesDgm; // first byte past end of data

                                                   // safety check - verify pointer is valid
      if (pData >= pEOD)
      {
         pData = NULL;
      }
   }

   return pData;
}

char *getMRZSoundings(void* tgm)
{
   char *pData = NULL;

   // if this record is not empty
   if (tgm != NULL)
   {
      pEMdgmMRZ dgm = (pEMdgmMRZ)tgm;
      int sizeCommon = dgm->cmnPart.numBytesCmnPart;

      // Find rxInfo. Must take account of number of TX sectors.
      int numTxSectors = dgm->pingInfo.numTxSectors;

      //Get PingInfo
      char* pPI = (char*)(&(dgm->partition.dgmNum));
      pPI += 2;
      pPI += sizeCommon;

      pEMdgmMRZ_pingInfo pPingInfo = (pEMdgmMRZ_pingInfo)pPI;

      //Move to end of pingInfo
      char* pch = pPI + pPingInfo->numBytesInfoData; //At start of txsectors
      int nSkip = 0;
      pch += nSkip;

      //Get sector
      int nBytesTx = pPingInfo->numBytesPerTxSector;
      pEMdgmMRZ_txSectorInfo pSect = (pEMdgmMRZ_txSectorInfo)pch;

      //Move to start of rxInfo
      pch = (char*)pSect;
      pch += (nBytesTx * numTxSectors);
      EMdgmMRZ_rxInfo* pRxInfo = (EMdgmMRZ_rxInfo*)pch;
      int nSizeRXInfo = pRxInfo->numBytesRxInfo;

      // Now, find soundings.
      int numExtraDet = pRxInfo->numExtraDetectionClasses;
      pch += nSizeRXInfo;
      pch += numExtraDet * sizeof(EMdgmMRZ_extraDetClassInfo);
      EMdgmMRZ_sounding* pSoundings = (EMdgmMRZ_sounding*)pch;

      pData = pch;

      // end of datagram for safety testing
      char *pSOD = (char*)dgm;
      char *pEOD = pSOD + dgm->header.numBytesDgm; // first byte past end of data

                                                   // safety check - verify pointer is valid
      if (pData >= pEOD)
      {
         pData = NULL;
      }
   }

   return pData;
}



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
				//unsigned char *buffer;
				
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
		//meh
	}
	
	else if(datagramType == "#IOP"){
		//meh
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
		processSCL(header, datagram);
	}
	
	else if(datagramType == "#SDE"){
		processSDE(header, datagram);
	}
	
	else if(datagramType == "#SHI"){
		processSHI(header, datagram);
	}
	
	else if(datagramType == "#MRZ"){
		// mbes ping
		processMRZ(header, datagram);
		exit(1);
	}
	
	else if(datagramType == "#MWC"){
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
	
	//std::cerr << svp.numBytesCmnPart <<"\n";
	
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

void KmallParser::processMRZ(EMdgmHeader & header, unsigned char * datagram){

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
	//EMdgmMRZ_txSectorInfo *sect = (EMdgmMRZ_txSectorInfo)p;
	
	p += (nBytesTx * numTxSectors);
	
	EMdgmMRZ_rxInfo *rxInfo = (EMdgmMRZ_rxInfo*)p;
	
	std::cerr<<"numSoundingsMaxMain: " << rxInfo->numSoundingsMaxMain << "\n";
	
	uint32_t nbSoundings = rxInfo->numSoundingsMaxMain;
	
	EMdgmMRZ_sounding_def soundings[MAX_NUM_BEAMS+MAX_EXTRA_DET] = {mrz.sounding};
	
	for(uint32_t i = 0; i<nbSoundings; i++){
		
		std::cerr<<"index: " << soundings[i]->soundingIndex <<"\n";
		std::cerr<<"twoWayTravelTime_sec: " << soundings[i]->twoWayTravelTime_sec <<"\n";
		std::cerr<<"twoWayTravelTimeCorrection_sec: " << soundings[i]->twoWayTravelTimeCorrection_sec <<"\n";
		std::cerr<<"beamAngleReRx_deg: " << soundings[i]->beamAngleReRx_deg <<"\n";
		std::cerr<<"qualityFactor: " << soundings[i]->qualityFactor <<"\n";
		break;
	}
	
	//__________________________________________________________________
	
	/*
	p = (unsigned char*)&mrz;
	
	//Structs above rxinfo can grow in newer datagram version. Use getMRZRxInfo to move to the correct location
    pEMdgmMRZ_rxInfo rxInfo = (pEMdgmMRZ_rxInfo)getMRZRxInfo(p);
	char* pData = (char*)(&(mrz.sectorInfo[mrz.pingInfo.numTxSectors]));
	
    //Structs above sounding can grow in newer datagram version. Use getMRZSoundings to move to the correct location
	pEMdgmMRZ_sounding depthList = (pEMdgmMRZ_sounding)getMRZSoundings(p);
	
	std::cerr<< "rxInfo->numSoundingsMaxMain: " << rxInfo->numSoundingsMaxMain <<"\n";
	
	for (int i = 0; i < rxInfo->numSoundingsMaxMain; i++) 
	{
		std::cerr<<"soundingIndex: " << (depthList+i)->soundingIndex <<"\n";
		std::cerr<<"qualityFactor: " << (depthList+i)->qualityFactor <<"\n";
		std::cerr<<"beamAngleReRx_deg: " << (depthList+i)->beamAngleReRx_deg <<"\n";
		std::cerr<<"twoWayTravelTime_sec: " << (depthList+i)->twoWayTravelTime_sec <<"\n";
		std::cerr<<"beamIncAngleAdj_deg: " << (depthList+i)->beamIncAngleAdj_deg <<"\n\n";
	}
	*/

}

void KmallParser::processSKM(EMdgmHeader & header, unsigned char * datagram){

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

void KmallParser::processSCL(EMdgmHeader & header, unsigned char * datagram){

	EMdgmSCL_def scl;	
	unsigned char *p;
	
	memset(&scl, 0, sizeof(EMdgmSCL_def)); // XXX not sure if essential
	
	p = (unsigned char*)&scl;
	
	memcpy(p, (char*)&header, sizeof(header));
	p+=sizeof(EMdgmHeader);
	
	memcpy(p, datagram, header.numBytesDgm-sizeof(EMdgmHeader));
	
	//TODO
	/*
	std::cerr<<scl.cmnPart.numBytesCmnPart<<"\n";
	std::cerr<<scl.sensData.offset_sec<<"\n";
	*/
}


void KmallParser::processSVT(EMdgmHeader & header, unsigned char * datagram){
	EMdgmSVT_def svt;	
	unsigned char *p;
	
	memset(&svt, 0, sizeof(EMdgmSVT_def)); // XXX not sure if essential
	
	p = (unsigned char*)&svt;
	
	memcpy(p, (char*)&header, sizeof(header));
	p+=sizeof(EMdgmHeader);
	
	memcpy(p, datagram, header.numBytesDgm-sizeof(EMdgmHeader));
	
	
	//TODO
	/*
	std::cerr<<svt.infoPart.numBytesCmnPart<<"\n";
	std::cerr<<svt.sensorData[0].time_sec<<"\n";
	*/
}

void KmallParser::processSDE(EMdgmHeader & header, unsigned char * datagram){
	EMdgmSDE_def sde;	
	unsigned char *p;
	
	memset(&sde, 0, sizeof(EMdgmSDE_def)); // XXX not sure if essential
	
	p = (unsigned char*)&sde;
	
	memcpy(p, (char*)&header, sizeof(header));
	p+=sizeof(EMdgmHeader);
	
	memcpy(p, datagram, header.numBytesDgm-sizeof(EMdgmHeader));
}

void KmallParser::processSHI(EMdgmHeader & header, unsigned char * datagram){
	EMdgmSHI_def shi;	
	unsigned char *p;
	
	memset(&shi, 0, sizeof(EMdgmSHI_def)); // XXX not sure if essential
	
	p = (unsigned char*)&shi;
	
	memcpy(p, (char*)&header, sizeof(header));
	p+=sizeof(EMdgmHeader);
	
	memcpy(p, datagram, header.numBytesDgm-sizeof(EMdgmHeader));
	
	//TODO
}


#endif
