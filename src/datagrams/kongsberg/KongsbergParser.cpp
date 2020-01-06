/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef KONGSBERG_CPP
#define KONGSBERG_CPP

#include "KongsbergParser.hpp"


KongsbergParser::KongsbergParser(DatagramEventHandler & processor):DatagramParser(processor){

}

KongsbergParser::~KongsbergParser(){

}

void KongsbergParser::parse(std::string & filename){
  FILE * file = fopen(filename.c_str(),"rb");

  if(file){
    while(!feof(file)){
      //Read datagramHeader
      KongsbergHeader hdr;
      int elementsRead = fread (&hdr,sizeof(KongsbergHeader),1,file);

      if(elementsRead == 1){
        //Check for starting character in datagram
        if(hdr.stx==STX){
          //Allocate memory for the datagram's content
          unsigned char * buffer = (unsigned char*)malloc(hdr.size-sizeof(KongsbergHeader)+sizeof(uint32_t));

          elementsRead = fread(buffer,hdr.size-sizeof(KongsbergHeader)+sizeof(uint32_t),1,file);

          processDatagram(hdr,buffer);

          free(buffer);
        }
        else{
          printf("%02x",hdr.size);
          throw new Exception("Bad datagram");
          //TODO: reject bad datagram, maybe log it
        }
      }
    }

    fclose(file);
  }
  else{
    throw new Exception("Couldn't open file " + filename);
  }
}

std::string KongsbergParser::getName(int tag)
{
  switch(tag)
  {

    case 48:
    return "PU Id output datagrams";
    break;

    case 49:
    return "PU Status output";
    break;

    case 51:
    return "ExtraParameters 3";
    break;

    case 53:
    return "Seabed image datagram";
    break;

    case 65:
    return "Attitude datagram";
    break;

    case 66:
    return "PU BIST result output";
    break;

    case 67:
    return "Clock datagrams";
    break;

    case 68:
    return "Depth datagram";
    break;

    case 69:
    return "Single beam echo sounder depth datagram";
    break;

    case 70:
    return "Raw range and beam angle datagrams";
    break;

    case 71:
    return "Surface sound speed datagram";
    break;

    case 72:
    return "Heading datagrams";
    break;

    case 73:
    return "Installation parameters";
    break;

    case 74:
    return "Mechanical transducer tilt datagrams";
    break;

    case 75:
    return "Central beams echogram";
    break;

    case 78:
    return "Raw range and beam angle 78 datagram";
    break;

    case 79:
    return "Quality factor datagram 79";
    break;

    case 80:
    return "Position datagrams";
    break;

    case 82:
    return "Runtime parameters";
    break;

    case 84:
    return "Tide datagram";
    break;

    case 85:
    return "Sound speed profile datagram";
    break;

    case 87:
    return "Kongsberg Maritime SSP output datagram";
    break;

    case 88:
    return "XYZ 88";
    break;

    case 89:
    return "Seabed image data 89 datagram";
    break;

    case 102:
    return "Raw range and beam angle datagrams";
    break;

    case 104:
    return "Depth (pressure) or height datagram";
    break;

    case 105:
    return "Installation parameters";
    break;

    case 107:
    return "Water column datagram";
    break;

    case 108:
    return "Extra detections";
    break;

    case 110:
    return "Network attitude velocity datagram 110";
    break;

    case 114:
    return "Installation parameters or remote information";
    break;

    default:
    return "Invalid tag";
    break;
  }
}

void KongsbergParser::processDatagram(KongsbergHeader & hdr,unsigned char * datagram){

  /*
  printf("-------------------------------------\n");
  printf("Datagram has %d bytes\n",hdr.size);
  printf("Datagram type: %c\n",hdr.type);
  printf("EM Model number: %d\n",hdr.modelNumber);
  printf("Date: %d\n",hdr.date);
  printf("Seconds since midnight: %d\n",hdr.time);
  printf("Counter: %d\n",hdr.counter);
  printf("Serial number: %d\n",hdr.serialNumber);
  */

  processor.processDatagramTag(hdr.type);

  switch(hdr.type){
    case 'A':
    processAttitudeDatagram(hdr,datagram);
    break;

    case 'D':
    processDepth(hdr,datagram);
    break;

    case 'E':
    //process echosounder data
    //processDepth(hdr,datagram);
    break;

    case 'N':
    processRawRangeAndBeam78(hdr,datagram);
    break;

    case 'O':
    //processQualityFactor(hdr,datagram);
    break;

    case 'P':
    processPositionDatagram(hdr,datagram);
    break;

    case 'h':
    //processWaterHeight(hdr,datagram);
    break;

    case 'U':
    processSoundSpeedProfile(hdr,datagram);
    break;

    case 'Y':
    //processSeabedImageData(hdr,datagram);
    break;

    default:
    //printf("Unknown type %c\n",hdr.type);
    break;
  }
}

void KongsbergParser::processDepth(KongsbergHeader & hdr,unsigned char * datagram){
  //printf("TODO: parse depth data\n");
}

void KongsbergParser::processWaterHeight(KongsbergHeader & hdr,unsigned char * datagram){
  //printf("TODO: parse height data\n");
}

void KongsbergParser::processAttitudeDatagram(KongsbergHeader & hdr,unsigned char * datagram){
  uint64_t microEpoch = convertTime(hdr.date,hdr.time);

  uint16_t nEntries = ((uint16_t*)datagram)[0];

  KongsbergAttitudeEntry * p = (KongsbergAttitudeEntry*) ((unsigned char*)datagram + sizeof(uint16_t));

  for(unsigned int i = 0;i<nEntries;i++) {
    double heading = (double)p[i].heading/(double)100;
    double pitch   = (double)p[i].pitch/(double)100;
    double roll    = (double)p[i].roll/(double)100;

    processor.processAttitude(
      microEpoch + p[i].deltaTime * 1000,
      heading,
      (pitch<0)?pitch+360:pitch,
      (roll<0)?roll+360:roll
    );
  }
}

void KongsbergParser::processSoundSpeedProfile(KongsbergHeader & hdr,unsigned char * datagram){
  SoundVelocityProfile * svp = new SoundVelocityProfile();
  uint64_t microEpoch = convertTime(hdr.date,hdr.time);

  KongsbergSoundSpeedProfile * ssp = (KongsbergSoundSpeedProfile*) datagram;

  if((ssp->profileDate != 0)&&(ssp->profileTime != 0))
  {
    microEpoch = convertTime(ssp->profileDate,ssp->profileTime);
  }

  svp->setTimestamp(microEpoch);

  KongsbergSoundSpeedProfileEntry * entry = (KongsbergSoundSpeedProfileEntry*)((unsigned char*)(&ssp->depthResolution)+sizeof(uint16_t));

  for(unsigned int i = 0;i< ssp->nbEntries;i++){
    double depth = (double)entry[i].depth / ((double)100 / (double)ssp->depthResolution );
    double soundSpeed = (double) entry[i].soundSpeed / (double) 10; //speed is in dm/s

    svp->add(depth,soundSpeed);
  }

  processor.processSoundVelocityProfile(svp);
}

uint64_t KongsbergParser::convertTime(uint32_t datagramDate,uint32_t datagramTime){
  int year = datagramDate / 10000;
  int month = (datagramDate - (year * 10000))/100;
  int day = datagramDate - (year * 10000) - (month * 100);

  //month is 1-12, day is 1-31
  return TimeUtils::build_time(year,month,day,datagramTime);
}

void KongsbergParser::processPositionDatagram(KongsbergHeader & hdr,unsigned char * datagram){
  KongsbergPositionDatagram * p = (KongsbergPositionDatagram*) datagram;

  uint64_t microEpoch = convertTime(hdr.date,hdr.time);

  //printf("%s",p->inputDatagram);

  double longitude = (double)p->longitude/(double)20000000;
  double latitude  = (double)p->lattitude/(double)20000000;

  std::string inputDatagram(p->inputDatagram);

  double height = std::numeric_limits<double>::quiet_NaN();

  //Extract ellipsoidal height from input datagram
  if(inputDatagram.find("GGK") != std::string::npos){
    height = NmeaUtils::extractHeightFromGGK(inputDatagram);

  }
  else if(inputDatagram.find("GGA") != std::string::npos){
    height = NmeaUtils::extractHeightFromGGA(inputDatagram);
  }
  else{
    //NO POSITION, whine about this
    std::cerr << "No ellipsoidal height found in input datagram: " << inputDatagram << std::endl;
  }

  if(!std::isnan(height)){
    processor.processPosition(microEpoch,longitude,latitude,height);
  }
}

void KongsbergParser::processQualityFactor(KongsbergHeader & hdr,unsigned char * datagram){
  //printf("TODO: parse quality factor data\n");
}

void KongsbergParser::processSeabedImageData(KongsbergHeader & hdr,unsigned char * datagram){
  //printf("TODO: parse Seabed Image Data\n");
}

void KongsbergParser::processRawRangeAndBeam78(KongsbergHeader & hdr,unsigned char * datagram){
  KongsbergRangeAndBeam78 * data = (KongsbergRangeAndBeam78*)datagram;

  uint64_t microEpoch = convertTime(hdr.date,hdr.time);

  processor.processSwathStart((double)data->surfaceSoundSpeed / (double)10);

  std::map<int,KongsbergRangeAndBeam78TxEntry*>  txEntries;

  KongsbergRangeAndBeam78TxEntry* tx = (KongsbergRangeAndBeam78TxEntry*) (((unsigned char *)data)+sizeof(KongsbergRangeAndBeam78));

  for(unsigned int i=0;i< data->nbTxPackets; i++){
    txEntries[tx[i].txSectorNumber] = &tx[i];
    //printf("Tilt: %0.2f\n",(double)tx[i].tiltAngle/(double)100);
  }

  KongsbergRangeAndBeam78RxEntry * rx = (KongsbergRangeAndBeam78RxEntry*)    ((((unsigned char *)data)+sizeof(KongsbergRangeAndBeam78)) + (data->nbTxPackets * sizeof(KongsbergRangeAndBeam78TxEntry)));

  for(unsigned int i=0;i<data->nbRxPackets;i++){
    //We'll hack-in the the beam angle as ID...Hail Satan!
    processor.processPing(microEpoch,rx[i].beamAngle,(double)rx[i].beamAngle/(double)100,(double)txEntries[rx[i].txSectorNumber]->tiltAngle/(double)100,rx[i].twoWayTravelTime,rx[i].qualityFactor,rx[i].reflectivity * 0.5);
  }
}

#endif