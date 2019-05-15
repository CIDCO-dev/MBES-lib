/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/*
* \author ?
*/

#ifndef MAIN_CPP
#define MAIN_CPP

#include "../datagrams/DatagramParserFactory.hpp"
#include <iostream>
#include <string>

/**Writes the usage information about the cidco-decoder*/
void printUsage(){
	std::cerr << "\n\
	NAME\n\n\
	cidco-decoder - lit un fichier MBES et le transforme en format cidco (ASCII)\n\n\
	SYNOPSIS\n \
	cidco-decoder fichier\n\n\
	DESCRIPTION\n\n \
	Copyright 2018 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés" << std::endl;
	exit(1);
}

/*!
* \brief Datagram printer class.
* \author ?
*
* Extention of Datagram processor class
*/
class DatagramPrinter : public DatagramEventHandler{
private:

	/**The heading File*/
	FILE *headingFile = NULL;

	/**The Pitch Roll File*/
	FILE *pitchRollFile = NULL;

	/**The position File*/
	FILE *positionFile = NULL;

	/**The multi beam File*/
	FILE *multibeamFile = NULL;

	/**The current timestamp*/
	uint64_t          currentMicroEpoch;

	/**The current Surface sound speed*/
	double            currentSurfaceSoundSpeed;

	/**Text value who the information of the pings*/
	std::stringstream pingLine;

	/**Number of beams*/
	int	          nbBeams = 0;

	/**Number of sound velocity profile*/
	int		  svpCount = 0;

public:
	/**
	* Create a datagram printer and open all the files
	*/
	DatagramPrinter(){
		headingFile = fopen("Heading.txt","w");
		pitchRollFile = fopen("PitchRoll.txt","w");
		positionFile = fopen("AntPosition.txt","w");
		multibeamFile = fopen("Multibeam.txt","w");
		pingLine.precision(10);
	}

	/**Destroy the datagram printer and close all the files*/
	~DatagramPrinter(){
		//last pingLine didnt get printed
		fprintf(multibeamFile,"%.6f\t%0.7f\t%d%s\n",microEpoch2daySeconds(currentMicroEpoch), currentSurfaceSoundSpeed, nbBeams,pingLine.str().c_str());

		fclose(headingFile);
		fclose(pitchRollFile);
		fclose(positionFile);
		fclose(multibeamFile);
	}

	/**
	* Add the information of a attitude on the pitchRollFile and headingFile
	*
	* @param microEpoch the attitude timestamp
	* @param heading the attitude heading
	* @param pitch the attitude pitch
	* @param roll the attitude roll
	*/
	void processAttitude(uint64_t microEpoch,double heading,double pitch,double roll){
		//CIDCO file format separates these 2...
		fprintf(pitchRollFile, "%.6f\t%.10lf\t%.10lf\n",microEpoch2daySeconds(microEpoch),pitch,roll);
		fprintf(headingFile, "%.6f\t%.10lf\n",microEpoch2daySeconds(microEpoch),heading);
	};

	/**
	* Add the information of a position on the positionFile
	*
	* @param microEpoch the position timestamp
	* @param longitude the position longitude
	* @param latitude the position latitude
	* @param height the position ellipsoidal height
	*/
	void processPosition(uint64_t microEpoch,double longitude,double latitude,double height){
		fprintf(positionFile, "%.6f\t%.10lf\t%.10lf\t%.10lf\n",microEpoch2daySeconds(microEpoch),latitude,longitude,height);

	};

	/**
	* Add the information of a ping on the pingLine
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
		currentMicroEpoch = microEpoch;
		nbBeams++;
		pingLine << "\t" << twoWayTravelTime << "\t" << beamAngle << "\t" << tiltAngle;
	};

	/**
	* Add the information of a swath on the pingLine
	*
	* @param surfaceSoundSpeed the new current surface sound speed
	*/
	void processSwathStart(double surfaceSoundSpeed){
		currentSurfaceSoundSpeed = surfaceSoundSpeed;
		if(nbBeams > 0){
			std::string cleanPingLine = trim(pingLine.str());
			fprintf(multibeamFile,"%.6f\t%0.7f\t%d\t%s\n",microEpoch2daySeconds(currentMicroEpoch), surfaceSoundSpeed, nbBeams, cleanPingLine.c_str());
			pingLine.str(std::string());
			nbBeams=0;
		}
	};

	/**
	* Make a file who contain the informations of a sound velocity profile
	*
	* @param svp the sound velocity profile
	*/
	void processSoundVelocityProfile(SoundVelocityProfile * svp){
		std::stringstream filename;

		filename << "SVP-" <<svpCount << ".svp";

		std::string f= filename.str();

		svp->write(f);

		svpCount++;

		std::cout << "Writing to " << filename.str() << std::endl;
	}

	/**
	* return the convert microsecond to day second
	*
	* @param microEpoch the timestamp to convert
	*/
	double microEpoch2daySeconds(uint64_t microEpoch){
		uint64_t microsInDay = (uint64_t)1000000L *  (uint64_t)60L *  (uint64_t)60L *  (uint64_t)24L;
		return (double)(microEpoch % microsInDay)  / (double)1000000;
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
	_putenv("TZ");
	#endif

	if(argc != 2){
		printUsage();
	}
	std::string fileName(argv[1]);
	try{
		std::cerr << "Decoding " << fileName << std::endl;

		parser = DatagramParserFactory::build(fileName,printer);

		parser->parse(fileName);
	}
	catch(const char * error){
		std::cerr << "Error whille parsing " << fileName << ": " << error << std::endl;
	}
	if(parser) delete parser;
}
#endif
