/*
Copyright 2022 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/*
 author Patrick Charron-Morneau
 hydroblock 2.0 parser
*/


#ifndef HYDROBLOCK20PARSER_CPP
#define HYDROBLOCK20PARSER_CPP

#include "Hydroblock20Parser.hpp"

Hydroblock20Parser::Hydroblock20Parser(DatagramEventHandler & processor):DatagramParser(processor){

}

Hydroblock20Parser::~Hydroblock20Parser(){

}

void Hydroblock20Parser::parse(std::string & dirPath, bool ignoreChecksum ){

	std::string gnssFilePath, imuFilePath, sonarFilePath;
	
	for (auto const& dir_entry : std::filesystem::directory_iterator(std::filesystem::path(dirPath))) {
		//std::cout << dir_entry.path().filename() << '\n';
		std::string filename = dir_entry.path().filename();
		
		//std::cerr<<filename.substr(18,3) <<"\n";
		
		if(filename.substr(18,3) == "imu"){
			imuFilePath = dir_entry.path();
		}
		else if(filename.substr(18,4) == "gnss"){
			gnssFilePath = dir_entry.path();
		}
		else if(filename.substr(18,5) == "sonar"){
			sonarFilePath = dir_entry.path();
		}
		else{
			std::cerr<<"invalid file \n";
		}
	}

	//std::cerr<<gnssFilePath<<" "<<imuFilePath << " " << sonarFilePath <<"\n";
	
	FILE *gnssFile, *imuFile, *sonarFile;

	char gnssBuff[70], imuBuff[70], sonarBuff[70];
	
	gnssFile = fopen(gnssFilePath.c_str(), "r");
	imuFile = fopen(imuFilePath.c_str(), "r");
	sonarFile = fopen(sonarFilePath.c_str(), "r");

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
	
	

	double lon, lat, ellipsoidalHeight, heading, pitch, roll, depth;
	int year,month,day,hour,minute,second,microSec,status, service;
	uint64_t microEpoch;
	bool gnssDone = false, imuDone = false, sonarDone = false;
	
	while(true){

		if(fgets(gnssBuff, 67, gnssFile)){
			
			
			fscanf(gnssFile, "%d-%d-%d %d:%d:%d.%d;%lf;%lf;%lf;%d;%d", &year, &month, &day, &hour, &minute, &second, &microSec, &lon, &lat, &ellipsoidalHeight, &status, &service);
			
			microEpoch = TimeUtils::build_time(year, month, day, hour, minute, second, microSec, 0);
			
			processor.processPosition(microEpoch, lon, lat, ellipsoidalHeight );
			
		}
		else{
			gnssDone = true;
		}
		
		
		if(fgets(imuBuff, 49, imuFile)){
		
			fscanf(imuFile, "%d-%d-%d %d:%d:%d.%d;%lf;%lf;%lf", &year, &month, &day, &hour, &minute, &second, &microSec, &heading, &pitch, &roll);
			microEpoch = TimeUtils::build_time(year, month, day, hour, minute, second, microSec, 0);	
			
			processor.processAttitude(microEpoch, heading, pitch, roll );
	
		}
		else{
			imuDone = true;
		}
		
		if(fgets(sonarBuff, 33, sonarFile)){
				
			fscanf(sonarFile, "%d-%d-%d %d:%d:%d.%d;%lf", &year, &month, &day, &hour, &minute, &second, &microSec, &depth);
			microEpoch = TimeUtils::build_time(year, month, day, hour, minute, second, microSec, 0);
			processor.processSwathStart(1500);
			processor.processPing(microEpoch, 0, 0.0, 0.0, depth/1500.0, 0, 0);
			
		}
		else{
			sonarDone = true;
		}
		
		if(gnssDone && imuDone && sonarDone){
			break;
		}
	
	}
}
#endif
