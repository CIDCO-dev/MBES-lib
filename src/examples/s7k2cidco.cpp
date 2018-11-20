/*
 *  Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */
#ifndef MAIN_CPP
#define MAIN_CPP

#include <getopt.h>

#include "../datagrams/kongsberg/KongsbergParser.hpp"
#include "../datagrams/xtf/XtfParser.hpp"
#include "../datagrams/s7k/S7kParser.hpp"
#include <iostream>
#include <string>
#include "../utils/StringUtils.hpp"

void printUsage(){
	std::cerr << "\n\
  NAME\n\n\
     s7k2cidco - lit un fichier s7k et le transforme en format cidco (ASCII)\n\n\
  SYNOPSIS\n \
	   datagram-dump fichier\n\n\
  DESCRIPTION\n\n \
  Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés" << std::endl;
	exit(1);
}

class DatagramPrinter : public DatagramProcessor{
    
        private:
                FILE *headingFile = NULL;
                FILE *pitchRollFile = NULL;
                FILE *positionFile = NULL;
                FILE *multibeamFile = NULL;
                
                uint64_t currentMicroEpoch = 0;
    
    
	public:
		DatagramPrinter(){
                    headingFile = fopen("Heading.txt","w");
                    pitchRollFile = fopen("PitchRoll.txt","w");
                    positionFile = fopen("AntPosition.txt","w");
                    multibeamFile = fopen("Multibeam.txt","w");
		}

		~DatagramPrinter(){
                    fclose(headingFile);
                    fclose(pitchRollFile);
                    fclose(positionFile);
                    fclose(multibeamFile);
		}
                
                

                void processAttitude(uint64_t microEpoch,double heading,double pitch,double roll){
                    //CIDCO file format separates these 2...
			fprintf(pitchRollFile, "A %lu %.10lf %.10lf\n",microEpoch,pitch,roll);
                        fprintf(headingFile, "A %lu %.10lf\n",microEpoch,heading);
		};

                void processPosition(uint64_t microEpoch,double longitude,double latitude,double height){
			fprintf(positionFile, "A %lu %.10lf %.10lf %.10lf\n",microEpoch,latitude,longitude,height);
		};

                void processPing(uint64_t microEpoch,long id, double beamAngle,double tiltAngle,double twoWayTravelTime,uint32_t quality,uint32_t intensity){
                    
		};

                void processSwathStart(){

		};
};


int main (int argc , char ** argv ){
	DatagramParser * parser = NULL;
	DatagramPrinter  printer;

	setenv("TZ", "UTC", 1);

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
