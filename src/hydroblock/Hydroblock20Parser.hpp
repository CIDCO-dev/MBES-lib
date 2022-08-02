#ifndef HYDROBLOCK20PARSER_HPP
#define HYDROBLOCK20PARSER_HPP

#include "../utils/TimeUtils.hpp"
#include "../datagrams/DatagramParser.hpp"
#include "../svp/SvpSelectionStrategy.hpp"
#include "../svp/SvpNearestByTime.hpp"
#include "../georeferencing/DatagramGeoreferencer.hpp"

/*
Copyright 2022 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/*
 author Patrick Charron-Morneau
 hydroblock 2.0 parser
*/

class Hydroblock20Parser : public DatagramParser{
	public:
				/**
                 * Create an XTF parser
                 *
                 * @param processor the datagram processor
                 */
		Hydroblock20Parser(DatagramEventHandler & processor):DatagramParser(processor){};

                /**Destroy the XTF parser*/
		~Hydroblock20Parser(){};

                /**
                 * Parse an XTF file
                 *
                 * @param filename name of the file to read
                 */
		void parse(std::string & gnssFilename, std::string & imuFilename, std::string & sonarFilename ){
			
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
					
					processor.processPing(microEpoch, 0, 0, 0, (depth/1450.0)/2.0, 3.14159, 1.61803);
					
				}
				else{
					sonarDone = true;
				}
				
				if(gnssDone && imuDone && sonarDone){
					break;
				}
			
			}
		}

};
#endif

