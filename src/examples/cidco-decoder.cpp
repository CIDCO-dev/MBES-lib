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
 class DatagramPrinter : public DatagramProcessor{
         private:
                FILE *headingFile = NULL;
                FILE *pitchRollFile = NULL;
                FILE *positionFile = NULL;
                FILE *multibeamFile = NULL;
 		uint64_t          currentMicroEpoch;
                double            currentSurfaceSoundSpeed;
		std::stringstream pingLine;
		int	          nbBeams = 0;
	public:
		DatagramPrinter(){
                    headingFile = fopen("Heading.txt","w");
                    pitchRollFile = fopen("PitchRoll.txt","w");
                    positionFile = fopen("AntPosition.txt","w");
                    multibeamFile = fopen("Multibeam.txt","w");
                    pingLine.precision(10);
		}
 		~DatagramPrinter(){
		    //last pingLine didnt get printed
                    fprintf(multibeamFile,"%.6f\t%0.7f\t%d%s\n",microEpoch2daySeconds(currentMicroEpoch), currentSurfaceSoundSpeed, nbBeams,pingLine.str().c_str());
		    
 	            fclose(headingFile);
                    fclose(pitchRollFile);
                    fclose(positionFile);
                    fclose(multibeamFile);
		}
                 void processAttitude(uint64_t microEpoch,double heading,double pitch,double roll){
                	//CIDCO file format separates these 2...
			fprintf(pitchRollFile, "%.6f\t%.10lf\t%.10lf\n",microEpoch2daySeconds(microEpoch),pitch,roll);
                        fprintf(headingFile, "%.6f\t%.10lf\n",microEpoch2daySeconds(microEpoch),heading);
		};
                 void processPosition(uint64_t microEpoch,double longitude,double latitude,double height){
			fprintf(positionFile, "%.6f\t%.10lf\t%.10lf\t%.10lf\n",microEpoch2daySeconds(microEpoch),latitude,longitude,height);

		};
                 void processPing(uint64_t microEpoch,long id, double beamAngle,double tiltAngle,double twoWayTravelTime,uint32_t quality,uint32_t intensity){
			currentMicroEpoch = microEpoch;
			nbBeams++;
			pingLine << "\t" << twoWayTravelTime << "\t" << beamAngle << "\t" << tiltAngle;
		};
                 void processSwathStart(double surfaceSoundSpeed){
			currentSurfaceSoundSpeed = surfaceSoundSpeed;
 			if(nbBeams > 0){
				std::string cleanPingLine = trim(pingLine.str());
				fprintf(multibeamFile,"%.6f\t%0.7f\t%d\t%s\n",microEpoch2daySeconds(currentMicroEpoch), surfaceSoundSpeed, nbBeams, cleanPingLine.c_str());
				pingLine.str(std::string());
				nbBeams=0;
			}
		};
 		double microEpoch2daySeconds(uint64_t microEpoch){
			uint64_t microsInDay = (uint64_t)1000000L *  (uint64_t)60L *  (uint64_t)60L *  (uint64_t)24L;
			return (double)(microEpoch % microsInDay)  / (double)1000000;
		}
 };
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
