/*
 *  Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */
#ifndef GEOREFERENCE_CPP
#define GEOREFERENCE_CPP

#include "../datagrams/kongsberg/KongsbergParser.hpp"
#include "../datagrams/xtf/XtfParser.hpp"
#include "../datagrams/s7k/S7kParser.hpp"
#include <iostream>
#include <string>
#include "../utils/StringUtils.hpp"
#include "../Ping.hpp"
#include "../Attitude.hpp"
#include "../math/Interpolation.hpp"

void printUsage(){
	std::cerr << "\n\
  NAME\n\n\
     georeference - Produit un nuage de points d'un fichier de datagrammes multifaisceaux\n\n\
  SYNOPSIS\n \
	   georeference fichier\n\n\
  DESCRIPTION\n\n \
  Copyright 2017-2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés" << std::endl;
	exit(1);
}

class DatagramGeoreferencer : public DatagramProcessor{
	public:
		DatagramGeoreferencer(){

		}

		~DatagramGeoreferencer(){

		}


                void processAttitude(uint64_t microEpoch,double heading,double pitch,double roll){
			attitudes.push_back(Attitude(microEpoch,roll,pitch,heading));
                };

                void processPosition(uint64_t microEpoch,double longitude,double latitude,double height){
			positions.push_back(Position(microEpoch,longitude,latitude,height));
                };

                void processPing(uint64_t microEpoch,long id, double beamAngle,double tiltAngle,double twoWayTravelTime,uint32_t quality,uint32_t intensity){
			pings.push_back(Ping(microEpoch,id,quality,intensity,currentSurfaceSoundSpeed,twoWayTravelTime,tiltAngle,beamAngle));
                };

                void processSwathStart(double surfaceSoundSpeed){
			currentSurfaceSoundSpeed = surfaceSoundSpeed;
                };

		void georeference(){
			//interpolate attitudes and positions around pings
			unsigned int attitudeIndex=0;
			unsigned int positionIndex=0;

			for(auto i=pings.begin();i!=pings.end();i++){
				while(attitudeIndex < attitudes.size() && attitudes[attitudeIndex+1].getTimestamp() < (*i).getTimestamp()){
					attitudeIndex++;
				}

				if(attitudeIndex == attitudes.size() - 1  && attitudes[attitudeIndex+1].getTimestamp() < (*i).getTimestamp()){
					break;
				}

				while(positionIndex < positions.size() && positions[positionIndex+1].getTimestamp() < (*i).getTimestamp()){
					positionIndex++;
				}

				if(positionIndex == positions.size() - 1  && positions[positionIndex+1].getTimestamp() < (*i).getTimestamp()){
					break;
				}

				Attitude & beforeAttitude = attitudes[attitudeIndex];
				Attitude & afterAttitude = attitudes[attitudeIndex+1];

				Position & beforePosition = positions[positionIndex];
				Position & afterPosition  = positions[positionIndex+1];

				std::cout << beforeAttitude << std::endl;
				std::cout << afterAttitude << std::endl;

				Attitude * interpolatedAttitude = Interpolator::interpolateAttitude(beforeAttitude,afterAttitude,(*i).getTimestamp());
				Position * interpolatedPosition = Interpolator::interpolatePosition(beforePosition,afterPosition,(*i).getTimestamp());

				//TODO: georef

				delete interpolatedAttitude;
				delete interpolatedPosition;
			}

		};

	private:
		double 			currentSurfaceSoundSpeed;
		std::vector<Ping> 	pings;
		std::vector<Position> 	positions;
		std::vector<Attitude> 	attitudes;


};

int main (int argc , char ** argv ){
	DatagramParser * parser = NULL;
	DatagramGeoreferencer  printer;

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

		printer.georeference();
	}
	catch(const char * error){
		std::cerr << "Error whille parsing " << fileName << ": " << error << std::endl;
	}


	if(parser) delete parser;
}


#endif
