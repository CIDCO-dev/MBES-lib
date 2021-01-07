/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/*
* \author Guillaume Labbe-Morissette
*/

#ifndef MAIN_CPP
#define MAIN_CPP

#include "../datagrams/DatagramParserFactory.hpp"
#include <iostream>
#include <string>

#pragma comment(lib, "Ws2_32.lib")

/**Writes the usage information about the datagram-dump*/
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
* \brief Datagram printer class.
*
* Extention of Datagram processor class
*/
class DatagramPrinter : public DatagramEventHandler {
public:
	/**
	* Creates a datagram printer and open all the files
	*/
	DatagramPrinter(){

	}

	/**Destroys the datagram printer and close all the files*/
	~DatagramPrinter(){

	}

	/**
	* Shows the information of an attitude
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
	* Shows the information of a position
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
	* Shows the information of a ping
	*
	* @param microEpoch the ping timestamp
	* @param id the ping id
	* @param beamAngle the ping beam angle
	* @param tiltAngle the ping tilt angle
	* @param twoWayTravelTime the ping two way travel time
	* @param quality the ping quality
	* @param intensity the ping intensity
	*/
	void processPing(uint64_t microEpoch,long id, double beamAngle,double tiltAngle,double twoWayTravelTime,uint32_t quality,int32_t intensity){
		printf("X %lu %lu %.10lf %.10lf %.10f %u %d\n",microEpoch,id,beamAngle,tiltAngle,twoWayTravelTime,quality,intensity);
	};

	/**
	* Shows the information of a swath
	*
	* @param surfaceSoundSpeed the new current surface sound speed
	*/
	void processSwathStart(double surfaceSoundSpeed){

	};
};

/**
* Declares the parser depending on argument received
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

		parser = DatagramParserFactory::build(fileName,printer);

		parser->parse(fileName, true);
	}
	catch(const char * error){
		std::cerr << "Error whille parsing " << fileName << ": " << error << std::endl;
	}


	if(parser) delete parser;
}


#endif
