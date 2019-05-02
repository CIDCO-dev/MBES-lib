#ifndef DATAGRAMGEOREFERENCER_HPP
#define DATAGRAMGEOREFERENCER_HPP
#include "Ping.hpp"
#include "Position.hpp"
#include "Attitude.hpp"
#include "Georeferencing.hpp"
#include "svp/SoundVelocityProfile.hpp"
#include "svp/SoundVelocityProfileFactory.hpp"
#include "datagrams/DatagramEventHandler.hpp"
#include "math/Interpolation.hpp"

/*!
 * \brief Datagramer Georeferencer class extention of the Datagram Processor class
 */
class DatagramGeoreferencer : public DatagramEventHandler{
        public:
                /**Create a datagram georeferencer*/
                DatagramGeoreferencer(){

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

                /**Return the georeference (the three ping, the quality and the intensity)*/
                void georeference(Eigen::Vector3d & leverArm){
                        //interpolate attitudes and positions around pings
                        unsigned int attitudeIndex=0;
                        unsigned int positionIndex=0;

			//Choose SVP
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
                                        std::cerr << "Using default model" << std::endl;
                                }
                        }


			//Georef pings
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

                                //georeference
                                Eigen::Vector3d georeferencedPing;
                                Georeferencing::georeference(georeferencedPing,*interpolatedAttitude,*interpolatedPosition,(*i),*svp,leverArm);

				processGeoreferencedPing(georeferencedPing,(*i).getQuality(),(*i).getIntensity());

                                delete interpolatedAttitude;
                                delete interpolatedPosition;
                 	}
                }

                virtual void processGeoreferencedPing(Eigen::Vector3d & georeferencedPing,uint32_t quality,int32_t intensity){
                        std::cout << georeferencedPing(0) << " " << georeferencedPing(1) << " " << georeferencedPing(2) << " " << quality  << " " << intensity << std::endl;
                }

        protected:

                /**the current surface sound speed*/
                double                                  currentSurfaceSoundSpeed;

                /**Vector of pings*/
                std::vector<Ping>                       pings;

                /**Vector of positions*/
                std::vector<Position>                   positions;

                /**vector of attitudes*/
                std::vector<Attitude>                   attitudes;

                /**vector of sound velocity profiles*/
                std::vector<SoundVelocityProfile*>      svps;
};

#endif
