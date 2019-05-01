/*
 * @author Guillaume Labbe-Morissette
 */


#ifndef KONGSBERG_CPP
#define KONGSBERG_CPP


#include <string>
#include <cstdio>
#include <iostream>
#include <cmath>

#include "../DatagramParser.hpp"
#include "../../utils/NmeaUtils.hpp"
#include "../../utils/TimeUtils.hpp"
#include "KongsbergTypes.hpp"

/*!
 * \brief Kongsberg parser class extention of Datagram parser class
 */
class KongsbergParser : public DatagramParser{
        public:
                
                /**
                 * Create an Kongsberg parser 
                 * 
                 * @param processor the datagram processor
                 */
	        KongsbergParser(DatagramEventHandler & processor);
                
                /**Destroy the Kongsberg parser*/
	        ~KongsbergParser();

	        //interface methods
                /**
                 * Read a file and change the Kongsberg parser depending on the information
                 * 
                 * @param filename name of the file to read
                 */
	        void parse(std::string & filename);

        private:

                /**
                 * Call the process datagram depending on the type of the Kongsberg Header
                 * 
                 * @param hdr the Kongsberg header
                 * @param datagram the datagram
                 */
	        void processDatagram(KongsbergHeader & hdr,unsigned char * datagram);
                
                /**
                 * call the process Depth
                 * 
                 * @param hdr the Kongsberg header
                 * @param datagram the datagram
                 */
	        void processDepth(KongsbergHeader & hdr,unsigned char * datagram);
                
                /**
                 * call the process Water Height
                 * 
                 * @param hdr the Kongsberg header
                 * @param datagram the datagram
                 */
	        void processWaterHeight(KongsbergHeader & hdr,unsigned char * datagram);
                
                /**
                 * call the process Attitude
                 * 
                 * @param hdr the Kongsberg header
                 * @param datagram the datagram
                 */
	        void processAttitudeDatagram(KongsbergHeader & hdr,unsigned char * datagram);
                
                /**
                 * call the process Position
                 * 
                 * @param hdr the Kongsberg header
                 * @param datagram the datagram
                 */
	        void processPositionDatagram(KongsbergHeader & hdr,unsigned char * datagram);
                
                /**
                 * call the process Quality Factor
                 * 
                 * @param hdr the Kongsberg header
                 * @param datagram the datagram
                 */
	        void processQualityFactor(KongsbergHeader & hdr,unsigned char * datagram);
                
                /**
                 * call the process Seabed Image Data
                 * 
                 * @param hdr the Kongsberg header
                 * @param datagram the datagram
                 */
	        void processSeabedImageData(KongsbergHeader & hdr,unsigned char * datagram);
                
                /**
                 * call the process Sound Speed Profile
                 * 
                 * @param hdr the Kongsberg header
                 * @param datagram the datagram
                 */
		void processSoundSpeedProfile(KongsbergHeader & hdr,unsigned char * datagram);

                /**
                 * Return in microsecond the timestamp
                 * 
                 * @param datagramDate the datagram date
                 * @param datagramTime the datagram time
                 */
	        long convertTime(long datagramDate,long datagramTime);

};

/**
 * Create an Kongsberg parser 
 * 
 * @param processor the datagram processor
 */
KongsbergParser::KongsbergParser(DatagramEventHandler & processor):DatagramParser(processor){

}

/**Destroy the Kongsberg parser*/
KongsbergParser::~KongsbergParser(){

}

/**
 * Read a file and change the Kongsberg parser depending on the information
 * 
 * @param filename name of the file to read
 */
void KongsbergParser::parse(std::string & filename){
	FILE * file = fopen(filename.c_str(),"rb");

	if(file){
		while(!feof(file)){
			//Lire datagramHeader
	        	KongsbergHeader hdr;
			int elementsRead = fread (&hdr,sizeof(KongsbergHeader),1,file);

			if(elementsRead == 1){
				//Verifier la presence du caractere de debut de trame
				if(hdr.stx==STX){
					//Allouer l'espace requis pour le contenu du datagramme
			                unsigned char * buffer = (unsigned char*)malloc(hdr.size-sizeof(KongsbergHeader)+sizeof(uint32_t));

                			elementsRead = fread(buffer,hdr.size-sizeof(KongsbergHeader)+sizeof(uint32_t),1,file);

					processDatagram(hdr,buffer);

					free(buffer);
				}
				else{
					printf("%02x",hdr.size);
					throw "Bad datagram";
					//TODO: rejct bad datagram, maybe log it
				}
            }
		}

		fclose(file);
	}
	else{
		throw "Couldn't open file " + filename;
	}
}

/**
 * Call the process datagram depending on the type of the Kongsberg Header
 * 
 * @param hdr the Kongsberg header
 * @param datagram the datagram
 */
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

/**
 * call the process Depth
 * 
 * @param hdr the Kongsberg header
 * @param datagram the datagram
 */
void KongsbergParser::processDepth(KongsbergHeader & hdr,unsigned char * datagram){
	//printf("TODO: parse depth data\n");
}

/**
 * call the process Water Height
 * 
 * @param hdr the Kongsberg header
 * @param datagram the datagram
 */
void KongsbergParser::processWaterHeight(KongsbergHeader & hdr,unsigned char * datagram){
    //printf("TODO: parse height data\n");
}

/**
 * call the process Attitude
 * 
 * @param hdr the Kongsberg header
 * @param datagram the datagram
 */
void KongsbergParser::processAttitudeDatagram(KongsbergHeader & hdr,unsigned char * datagram){
    uint64_t microEpoch = convertTime(hdr.date,hdr.time);

    uint16_t nEntries = ((uint16_t*)datagram)[0];

    KongsbergAttitudeEntry * p = (KongsbergAttitudeEntry*) ((unsigned char*)datagram + sizeof(uint16_t));

    for(unsigned int i = 0;i<nEntries;i++){
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

/**
 * call the process Sound speed profile
 * 
 * @param hdr the Kongsberg header
 * @param datagram the datagram
 */
void KongsbergParser::processSoundSpeedProfile(KongsbergHeader & hdr,unsigned char * datagram){
    SoundVelocityProfile * svp = new SoundVelocityProfile();

    KongsbergSoundSpeedProfile * ssp = (KongsbergSoundSpeedProfile*) datagram;

    uint64_t microEpoch = convertTime(ssp->profileDate,ssp->profileTime);

    KongsbergSoundSpeedProfileEntry * entry = (KongsbergSoundSpeedProfileEntry*)((unsigned char*)(&ssp->depthResolution)+sizeof(uint16_t));

    svp->setTimestamp(microEpoch);

    for(unsigned int i = 0;i< ssp->nbEntries;i++){
	double depth = (double)entry[i].depth / ((double)100 / (double)ssp->depthResolution );
	double soundSpeed = (double) entry[i].soundSpeed / (double) 10; //speed is in dm/s

	svp->add(depth,soundSpeed);
    }

    processor.processSoundVelocityProfile(svp);
}

/**
 * Return in microsecond the timestamp
 * 
 * @param datagramDate the datagram date
 * @param datagramTime the datagram time
 */
long KongsbergParser::convertTime(long datagramDate,long datagramTime){
    int year = datagramDate / 10000;
    int month = (datagramDate - (datagramDate / 10000))/100;
    int day = datagramDate - ((datagramDate - (datagramDate / 10000))/100);

    //month is 1-12, day is 1-31, shift to zero offset
    return TimeUtils::build_time(year,month-1,day-1,datagramTime);
}

/**
 * call the process Position
 * 
 * @param hdr the Kongsberg header
 * @param datagram the datagram
 */
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

/**
 * call the process Quality Factor
 * 
 * @param hdr the Kongsberg header
 * @param datagram the datagram
 */
void KongsbergParser::processQualityFactor(KongsbergHeader & hdr,unsigned char * datagram){
        //printf("TODO: parse quality factor data\n");
}

/**
 * call the process Seabed Image Data
 * 
 * @param hdr the Kongsberg header
 * @param datagram the datagram
 */
void KongsbergParser::processSeabedImageData(KongsbergHeader & hdr,unsigned char * datagram){
	//printf("TODO: parse Seabed Image Data\n");
}

#endif
