/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef DATAGRAMGEOREFERENCER_HPP
#define DATAGRAMGEOREFERENCER_HPP
#include "../Ping.hpp"
#include "../Position.hpp"
#include "../Attitude.hpp"
#include "Georeferencing.hpp"
#include "../svp/SoundVelocityProfile.hpp"
#include "../svp/SoundVelocityProfileFactory.hpp"
#include "../datagrams/DatagramEventHandler.hpp"
#include "../math/Interpolation.hpp"

/*!
* \brief Datagram Georeferencer class.
* \author Guillaume Labbe-Morissette, Jordan McManus, Emile Gagne
*
* Extention of the DatagramEventHandler class
*/
class DatagramGeoreferencer : public DatagramEventHandler{
        public:
                /**Create a datagram georeferencer*/
                DatagramGeoreferencer(Georeferencing & geo) : georef(geo){

                }

                /**Destroy the datagram georeferencer*/
                ~DatagramGeoreferencer(){

                }

                /**
                 * Add the information of a attitude in the vector attitudes
                 * 
                 * @param microEpoch the attitude timestamp
                 * @param heading the attitude heading
                 * @param pitch the attitude pitch
                 * @param roll the attitude roll
                 */
                void processAttitude(uint64_t microEpoch,double heading,double pitch,double roll){
                        attitudes.push_back(Attitude(microEpoch,roll,pitch,heading));
                };

                /**
                 * Add the information of a position in the vector positions
                 * 
                 * @param microEpoch the position timestamp
                 * @param longitude the position longitude
                 * @param latitude the position latitude
                 * @param height the position ellipsoidal height
                 */
                void processPosition(uint64_t microEpoch,double longitude,double latitude,double height){
                        positions.push_back(Position(microEpoch,longitude,latitude,height));
                };

                /**
                 * Add the information of a ping in the vector pings
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
                        pings.push_back(Ping(microEpoch,id,quality,intensity,currentSurfaceSoundSpeed,twoWayTravelTime,tiltAngle,beamAngle));
                };

                /**
                 * Change the current surface sound speed
                 * 
                 * @param surfaceSoundSpeed the new current surface sound speed
                 */
                void processSwathStart(double surfaceSoundSpeed){
                        currentSurfaceSoundSpeed = surfaceSoundSpeed;
                };

                /**
                 * Add a sound velocity profile in the vector svp
                 * 
                 * @param svp the sound velocity profile
                 */
                void processSoundVelocityProfile(SoundVelocityProfile * svp){
                        svps.push_back(svp);
                }

                /**
                 * Georeferences all pings
		 * @param boresight boresight (dPhi,dTheta,dPsi)
                 */
                void georeference(Eigen::Vector3d & leverArm,Eigen::Matrix3d & boresight,SoundVelocityProfile * svpFile){


			//Choose SVP
                        SoundVelocityProfile * svp = NULL;

			if(svpFile != NULL){
				//If file is specified, use that
				svp = svpFile;
				std::cerr << "[+] Using SVP file" << std::endl;
			}
                        else if(svps.size() == 1){
				//else if we have an SVP inside the sonar file, use that
	                        svp = svps[0];
				std::cerr << "[+] Using SVP from sonar file" << std::endl;
                        }
                        else{
	                        if(svps.size() > 0){
        	                        //TODO: use a strategy (nearest by time, nearest by location, etc)
                                        std::cerr << "[-] Multiple SVP mode not yet implemented" << std::endl;
                                        exit(1);
                                }
                                else{
                	                //Else use default model
                                        svp = SoundVelocityProfileFactory::buildSaltWaterModel();
                                        std::cerr << "[+] Using default SVP model" << std::endl;
                                }
                        }

			//If no centroid defined for LGF georeferencing, compute one
			if(GeoreferencingLGF * lgf = dynamic_cast<GeoreferencingLGF*>(&georef)){
				if(lgf->getCentroid() == NULL){
					Position centroid(0,0,0,0);

					for(auto i=positions.begin();i!=positions.end();i++){
						centroid.getVector() += i->getVector();
					}

					centroid.getVector() /= (double)positions.size();

					lgf->setCentroid(centroid);

					std::cerr << "[+] Centroid: " << centroid << std::endl;
				}
			}

			//Sort everything
			std::sort(positions.begin(),positions.end(),&Position::sortByTimestamp);
			std::sort(attitudes.begin(),attitudes.end(),&Attitude::sortByTimestamp);
			std::sort(pings.begin(),pings.end(),&Ping::sortByTimestamp);

                        fprintf(stderr,"[+] Position data points: %ld [%lu to %lu]\n",positions.size(),positions[0].getTimestamp(),positions[positions.size()-1].getTimestamp());
                        fprintf(stderr,"[+] Attitude data points: %ld [%lu to %lu]\n",attitudes.size(),attitudes[0].getTimestamp(),attitudes[attitudes.size()-1].getTimestamp());
                        fprintf(stderr,"[+] Ping data points: %ld [%lu to %lu]\n",pings.size(),(pings.size() >0)?pings[0].getTimestamp():0,(pings.size() >0)?pings[pings.size()-1].getTimestamp():0);
                        
                        //interpolate attitudes and positions around pings
                        unsigned int attitudeIndex=0;
                        unsigned int positionIndex=0;                        
                        
			//Georef pings
                        for(auto i=pings.begin();i!=pings.end();i++){

                            
                                while(attitudeIndex +1 < attitudes.size() && attitudes[attitudeIndex + 1].getTimestamp() < (*i).getTimestamp()){
                                        attitudeIndex++;
                                }

				//No more attitudes available
                                if(attitudeIndex >= attitudes.size() - 1){
                                        //std::cerr << "No more attitudes" << std::endl;
                                        break;
                                }

                                while(positionIndex +1 < positions.size() && positions[positionIndex + 1].getTimestamp() < (*i).getTimestamp()){
                                        positionIndex++;
                                }

				//No more positions available
                                if( positionIndex >= positions.size() - 1){
                                        //std::cerr << "No more positions" << std::endl;
                                        break;
                                }

				//No position or attitude smaller than ping, so discard this ping
				if(positions[positionIndex].getTimestamp() > (*i).getTimestamp() || attitudes[attitudeIndex].getTimestamp() > (*i).getTimestamp()){
                                    std::cerr << "rejecting ping " << (*i).getId() << " " << (*i).getTimestamp() << " " << positions[positionIndex].getTimestamp() << " " << attitudes[attitudeIndex].getTimestamp()<< std::endl;
                                    continue;
				}

                                Attitude & beforeAttitude = attitudes[attitudeIndex];
                                Attitude & afterAttitude = attitudes[attitudeIndex+1];

                                Position & beforePosition = positions[positionIndex];
                                Position & afterPosition  = positions[positionIndex+1];

                                Attitude * interpolatedAttitude = Interpolator::interpolateAttitude(beforeAttitude,afterAttitude,(*i).getTimestamp());
                                Position * interpolatedPosition = Interpolator::interpolatePosition(beforePosition,afterPosition,(*i).getTimestamp());

                                //georeference
                                Eigen::Vector3d georeferencedPing;
                                georef.georeference(georeferencedPing,*interpolatedAttitude,*interpolatedPosition,(*i),*svp,leverArm,boresight);

				processGeoreferencedPing(georeferencedPing,(*i).getQuality(),(*i).getIntensity(),positionIndex,attitudeIndex);

                                delete interpolatedAttitude;
                                delete interpolatedPosition;
                 	}
                }

                virtual void processGeoreferencedPing(Eigen::Vector3d & georeferencedPing,uint32_t quality,int32_t intensity,int positionIndex,int attitudeIndex){
                        std::cout << georeferencedPing(0) << " " << georeferencedPing(1) << " " << georeferencedPing(2) << " " << quality  << " " << intensity << " " << positionIndex << " " << attitudeIndex << std::endl;
                }

        protected:
		/**the georeferencing method */
		Georeferencing & georef;

                /**the current surface sound speed*/
                double                                  currentSurfaceSoundSpeed;

                /**Vector of pings*/
                std::vector<Ping>                       pings;

                /**Vector of positions*/
                std::vector<Position>                   positions;

                /**vector of attitudes*/
                std::vector<Attitude>                   attitudes;

                /**Vector of SoundVelocityProfile*/
                std::vector<SoundVelocityProfile*>      svps;
};

#endif
