/*
 *  Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */
#ifndef MAIN_CPP
#define MAIN_CPP

#include "../datagrams/kongsberg/KongsbergParser.hpp"
#include "../datagrams/xtf/XtfParser.hpp"
#include "../datagrams/s7k/S7kParser.hpp"
#include <iostream>
#include <string>
#include "../utils/StringUtils.hpp"

/**Write the information about the datagram-dump*/ 
void printUsage(){
	std::cerr << "\n\
  NAME\n\n\
     datagram-dump - lit un fichier binaire et le transforme en format texte (ASCII)\n\n\
  SYNOPSIS\n \
	   datagram-dump fichier\n\n\
  DESCRIPTION\n\n \
  Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés" << std::endl;
	exit(1);
}

/*!
  * \brief Datagram printer class extention of Datagram processor class
  */
class DatagramPrinter : public DatagramProcessor{
	public:
                /**
                 * Create a datagram printer and open all the files
                 */
		DatagramPrinter(){

		}
                
                /**Destroy the datagram printer and close all the files*/
		~DatagramPrinter(){

		}

                /**
                 * show the information of a attitude
                 * 
                 * @param microEpoch the attitude timestamp
                 * @param heading the attitude heading
                 * @param pitch the attitude pitch
                 * @param roll the attitude roll
                 */
                void processAttitude(uint64_t microEpoch,double heading,double pitch,double roll){
			printf("A %lu %.10lf %.10lf %.10lf\n",microEpoch,heading,pitch,roll);
		};

                /**
                 * show the information of a position
                 * 
                 * @param microEpoch the position timestamp
                 * @param longitude the position longitude
                 * @param latitude the position latitude
                 * @param height the position ellipsoidal height
                 */
                void processPosition(uint64_t microEpoch,double longitude,double latitude,double height){
			printf("P %lu %.12lf %.12lf %.12lf\n",microEpoch,longitude,latitude,height);
		};

                /**
                 * show the information of a ping
                 * 
                 * @param microEpoch the ping timestamp
                 * @param id the ping id
                 * @param beamAngle the ping beam angle
                 * @param tiltAngle the ping tilt angle
                 * @param twoWayTravelTime the ping two way travel time
                 * @param quality the ping quality
                 * @param intensity the ping intensity
                 */
                void processPing(uint64_t microEpoch,long id, double beamAngle,double tiltAngle,double twoWayTravelTime,uint32_t quality,uint32_t intensity){
			printf("X %lu %lu %.10lf %.10lf %.10f %u %u\n",microEpoch,id,beamAngle,tiltAngle,twoWayTravelTime,quality,intensity);
		};

                /**
                 * show the information of a swath
                 * 
                 * @param surfaceSoundSpeed the new current surface sound speed
                 */
                void processSwathStart(double surfaceSoundSpeed){

		};
};

/**
  * declare the parser depending on argument receive
  * 
  * @param argc number of argument
  * @param argv value of the arguments
  */
int main (int argc , char ** argv ){
	DatagramParser * parser = NULL;
	DatagramPrinter  printer;

#ifdef __GNU__
	setenv("TZ", "UTC", 1);
#endif
#ifdef _WIN32
	putenv("TZ");
#endif

	if(argc != 2){
		printUsage();
	}

	std::string fileName(argv[1]);

	try{
		std::cerr << "Decoding " << fileName << std::endl;

		if(ends_with(fileName.c_str(),".all")){
			parser = new KongsbergParser(printer);
		}
		else if(ends_with(fileName.c_str(),".xtf")){
			parser = new XtfParser(printer);
		}
		else if(ends_with(fileName.c_str(),".s7k")){
                        parser = new S7kParser(printer);
		}
		else{
			throw "Unknown extension";
		}

		parser->parse(fileName);
	}
	catch(const char * error){
		std::cerr << "Error whille parsing " << fileName << ": " << error << std::endl;
	}


	if(parser) delete parser;
}


#endif
