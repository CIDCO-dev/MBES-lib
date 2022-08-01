

// C
#include <stdio.h>
#include <stdlib.h>

// c++
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// MBES
#include "../Position.hpp"
#include "../Attitude.hpp"
#include "../georeferencing/Georeferencing.hpp"
#include "../math/Interpolation.hpp"
#include "../utils/TimeUtils.hpp"
/*
Copyright 2022 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/*
 author Patrick Charron-Morneau
 hydroblock 2.0 parser
*/


// Writes the usage information about the cidco-decoder
void printUsage(){
	std::cerr << "\n\
	NAME\n\n\
	hydroblock-reader - output en format cidco (ASCII)\n\n\
	SYNOPSIS\n \
	hydroblock-reader gps.txt imu.txt sonar.txt\n\n\
	DESCRIPTION\n\n \
	Copyright 2022 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés" << std::endl;
	exit(1);
}

/*
// microEpoch number of micro-seconds since January 1st 1970
int dateTime2MicroEpoch(year, month, day, hour, minute, second){

	
}
*/
int main (int argc , char ** argv ){

	
	if(argc != 4){
		printUsage();
	}
	std::string gnssFilename = argv[1];
	std::string imuFilename = argv[2];
	std::string sonarFilename = argv[3];

	
	FILE *gnssFile, *imuFile, *sonarFile;

	char gnssBuff[70], imuBuff[70], sonarBuff[70];
	
	gnssFile = fopen(gnssFilename.c_str(), "r");
	imuFile = fopen(imuFilename.c_str(), "r");
	sonarFile = fopen(sonarFilename.c_str(), "r");

	if (NULL == gnssFile) {
        std::cerr<<"gnss file can't be opened \n";
    }

	if (NULL == imuFile) {
        std::cerr<<"imu file can't be opened \n";
    }
    
    if (NULL == sonarFile) {
        std::cerr<<"sonar file can't be opened \n";
    }
    
    // skip headers
    fgets(gnssBuff, 62, gnssFile);
    fgets(imuBuff, 29, imuFile);
    fgets(sonarBuff, 16, sonarFile);
    /*
    std::string header(gnssBuff);
    std::cout<<header<<"\n";
	*/
	
	
	
	
	int count =0;
    
	while(fgets(gnssBuff, 67, gnssFile) || fgets(imuBuff, 49, imuFile) || fgets(sonarBuff, 33, sonarFile)){
		
		double lon, lat, ellipsoidalHeight, heading, pitch, roll, depth;
    	int year,month,day,hour,minute,second,microSec,status, service;
    	
		fscanf(gnssFile, "%d-%d-%d %d:%d:%d.%d;%lf;%lf;%lf;%d;%d", &year, &month, &day, &hour, &minute, &second, &microSec, &lon, &lat, &ellipsoidalHeight, &status, &service);
		
		uint64_t microEpoch = TimeUtils::build_time(year, month, day, hour, minute, second, microSec, 0);
		
		std::cout<< microEpoch<<"\n";
		Position pos(microEpoch, lat, lon, ellipsoidalHeight);
		std::cout<< lon << " " << lat << " " << ellipsoidalHeight << " " << status << " " << service << "\n";

		fscanf(imuFile, "%d-%d-%d %d:%d:%d.%d;%lf;%lf;%lf", &year, &month, &day, &hour, &minute, &second, &microSec, &heading, &pitch, &roll);
		microEpoch = TimeUtils::build_time(year, month, day, hour, minute, second, microSec, 0);	
		//std::cout<< microEpoch<<" : ";
		std::cout<< heading <<" "<<pitch<<" "<<roll<<"\n";

		fscanf(sonarFile, "%d-%d-%d %d:%d:%d.%d;%lf", &year, &month, &day, &hour, &minute, &second, &microSec, &depth);
		microEpoch = TimeUtils::build_time(year, month, day, hour, minute, second, microSec, 0);	
		//std::cout<< microEpoch<<" : ";
		std::cout<< depth <<"\n";
		
		count++;
	
	}
	std::cout<<"\n"<<count<<"\n";
	return 0;
	
	
}
