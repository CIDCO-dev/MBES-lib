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
#include "../Georeferencing.hpp"
#include "../SoundVelocityProfileFactory.hpp"

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

		void processSoundVelocityProfile(SoundVelocityProfile * svp){
			svps.push_back(svp);
		}

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

				Attitude * interpolatedAttitude = Interpolator::interpolateAttitude(beforeAttitude,afterAttitude,(*i).getTimestamp());
				Position * interpolatedPosition = Interpolator::interpolatePosition(beforePosition,afterPosition,(*i).getTimestamp());

				//get SVP for this ping
				SoundVelocityProfile * svp = NULL;

				if(svps.size() == 1){
					svp = svps[0];
				}
				else{
					if(svps.size() > 0){
						//TODO: use a strategy (nearest by time, nearest by location, etc)
						std::cerr << "Multiple SVP mode not yet implemented" << std::endl;
						exit(1);
					}
					else{
						//use default model
						//TODO: allow different models to be used with command line switches
						svp = SoundVelocityProfileFactory::buildSaltWaterModel();
					}
				}

				//TODO: Get leverArm from the data or command line
				Eigen::Vector3d leverArm;
				leverArm << 0,0,0;

				//georeference
				Eigen::Vector3d georeferencedPing;
				Georeferencing::georeference(georeferencedPing,*interpolatedAttitude,*interpolatedPosition,(*i),*svp,leverArm);

				std::cout << georeferencedPing(0) << " " << georeferencedPing(1) << " " << georeferencedPing(2) << std::endl;

				delete interpolatedAttitude;
				delete interpolatedPosition;
			}

		};

	private:
		double 					currentSurfaceSoundSpeed;
		std::vector<Ping> 			pings;
		std::vector<Position> 			positions;
		std::vector<Attitude> 			attitudes;
		std::vector<SoundVelocityProfile*>  	svps;

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


		std::cout << std::setprecision(6);
		std::cout << std::fixed;

		printer.georeference();
	}
	catch(const char * error){
		std::cerr << "Error whille parsing " << fileName << ": " << error << std::endl;
	}


	if(parser) delete parser;
}

#endif
