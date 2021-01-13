/*
 * Copyright 2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   DatagramRayTracer.hpp
 * Author: Jordan McManus
 */

#ifndef DATAGRAMRAYTRACER_HPP
#define DATAGRAMRAYTRACER_HPP

#include "../Ping.hpp"
#include "../Position.hpp"
#include "../Attitude.hpp"
#include "../datagrams/DatagramEventHandler.hpp"
#include "../georeferencing/Raytracing.hpp"
#include "../svp/SvpSelectionStrategy.hpp"
#include "../svp/SoundVelocityProfileFactory.hpp"
#include "../math/Interpolation.hpp"


/*!
* \brief Datagram ray tracer class.
*
* Extention of Datagram processor class
*/

class DatagramRayTracer : public DatagramEventHandler {
    
    
public:
    /**
    * Creates a datagram ray tracer and open all the files
    */
    DatagramRayTracer(SvpSelectionStrategy & svpStrat) : svpStrategy(svpStrat) {

    }

    /**Destroys the datagram printer and close all the files*/
    ~DatagramRayTracer(){

    }

    /**
    * Shows the information of an attitude
    *
    * @param microEpoch the attitude timestamp
    * @param heading the attitude heading
    * @param pitch the attitude pitch
    * @param roll the attitude roll
    */
    void processAttitude(uint64_t microEpoch,double heading,double pitch,double roll){
        attitudes.push_back(Attitude(microEpoch, roll, pitch, heading));
    };

    /**
    * Shows the information of a position
    *
    * @param microEpoch the position timestamp
    * @param longitude the position longitude
    * @param latitude the position latitude
    * @param height the position ellipsoidal height
    */
    void processPosition(uint64_t microEpoch,double longitude,double latitude,double height){
        positions.push_back(Position(microEpoch, latitude, longitude, height));
    };

    /**
    * Shows the information of a ping
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
        pings.push_back(Ping(microEpoch, id, quality, intensity, currentSurfaceSoundSpeed, twoWayTravelTime, tiltAngle, beamAngle));
    };
    
    /**
     * Change the current surface sound speed
     * 
     * @param surfaceSoundSpeed the new current surface sound speed
     */
    void processSwathStart(double surfaceSoundSpeed) {
        currentSurfaceSoundSpeed = surfaceSoundSpeed;
    };

    /**
     * Add a sound velocity profile in the vector svp
     * 
     * @param svp the sound velocity profile
     */
    void processSoundVelocityProfile(SoundVelocityProfile * svp) {
        svps.push_back(svp);
    }
    
    void raytrace(Eigen::Vector3d & leverArm, Eigen::Matrix3d & boresight, std::vector<SoundVelocityProfile*> & externalSvps) {
        if(positions.size()==0){
		std::cerr << "[-] No position data found in file" << std::endl;
		return;
	}

        if(attitudes.size()==0){
                std::cerr << "[-] No attitude data found in file" << std::endl;
                return;
        }

        if(pings.size()==0){
                std::cerr << "[-] No ping data found in file" << std::endl;
                return;
        }



        if (externalSvps.size() > 0) {
            //Use svps specified by user
            for (unsigned int i = 0; i < externalSvps.size(); ++i) {
                svpStrategy.addSvp(externalSvps[i]);
            }

            std::cerr << "[+] Using SVP file" << std::endl;
        } else if(svps.size() > 0) {
            //Use svps contained inside sonar file
            for (unsigned int i = 0; i < svps.size(); ++i) {
                svpStrategy.addSvp(svps[i]);
            }

            std::cerr << "[+] Using SVP from sonar file" << std::endl;
        } else {
            //Default to fresh water
            svpStrategy.addSvp(SoundVelocityProfileFactory::buildFreshWaterModel());
            std::cerr << "[+] Using default SVP model" << std::endl;
        }

        //Sort everything
        std::sort(positions.begin(), positions.end(), &Position::sortByTimestamp);
        std::sort(attitudes.begin(), attitudes.end(), &Attitude::sortByTimestamp);
        std::sort(pings.begin(), pings.end(), &Ping::sortByTimestamp);

        // For correct display of timestamps on Windows
        std::cerr <<  "[+] Position data points: " << positions.size() << " [" << positions[0].getTimestamp() << " to " 
                << positions[positions.size() - 1].getTimestamp() << "]\n";
        std::cerr <<  "[+] Attitude data points: " << attitudes.size() << " [" << attitudes[0].getTimestamp() << " to " 
                << attitudes[attitudes.size() - 1].getTimestamp() << "]\n";      
        std::cerr <<  "[+] Ping data points: " << pings.size() << " [" << ( (pings.size() > 0) ? pings[0].getTimestamp() : 0 ) << " to " 
                << ( (pings.size() > 0) ? pings[pings.size() - 1].getTimestamp() : 0 ) << "]\n";                            

        //interpolate attitudes and positions around pings
        unsigned int attitudeIndex = 0;
        unsigned int positionIndex = 0;

        //Georef pings
        for (auto i = pings.begin(); i != pings.end(); i++) {


            while (attitudeIndex + 1 < attitudes.size() && attitudes[attitudeIndex + 1].getTimestamp() < (*i).getTimestamp()) {
                attitudeIndex++;
            }

            //No more attitudes available
            if (attitudeIndex >= attitudes.size() - 1) {
                //std::cerr << "No more attitudes" << std::endl;
                break;
            }

            while (positionIndex + 1 < positions.size() && positions[positionIndex + 1].getTimestamp() < (*i).getTimestamp()) {
                positionIndex++;
            }

            //No more positions available
            if (positionIndex >= positions.size() - 1) {
                //std::cerr << "No more positions" << std::endl;
                break;
            }

            //No position or attitude smaller than ping, so discard this ping
            if (positions[positionIndex].getTimestamp() > (*i).getTimestamp() || attitudes[attitudeIndex].getTimestamp() > (*i).getTimestamp()) {
                std::cerr << "rejecting ping " << (*i).getId() << " " << (*i).getTimestamp() << " " << positions[positionIndex].getTimestamp() << " " << attitudes[attitudeIndex].getTimestamp() << std::endl;
                continue;
            }

            Attitude & beforeAttitude = attitudes[attitudeIndex];
            Attitude & afterAttitude = attitudes[attitudeIndex + 1];

            Position & beforePosition = positions[positionIndex];
            Position & afterPosition = positions[positionIndex + 1];

            Attitude * interpolatedAttitude = Interpolator::interpolateAttitude(beforeAttitude, afterAttitude, (*i).getTimestamp());
            Position * interpolatedPosition = Interpolator::interpolatePosition(beforePosition, afterPosition, (*i).getTimestamp());
            
            // Set the transducer depth to draft
            // If we have timestamped vertical motion, then this would need to
            // be processed and interpolated in the same way as Position and Attitude
            i->setTransducerDepth(transducerDraft); // i is the i-th ping

            //raytracing
            Eigen::Matrix3d imu2nav;
            CoordinateTransform::getDCM(imu2nav, *interpolatedAttitude);
            
            Eigen::Vector3d rayTracedBeam;
            Raytracing::rayTrace(rayTracedBeam, (*i), *(svpStrategy.chooseSvp(*interpolatedPosition, *i)), boresight, imu2nav);
            

            processRayTracedBeam(rayTracedBeam);

            delete interpolatedAttitude;
            delete interpolatedPosition;
        }
    }
    
    virtual void processRayTracedBeam( Eigen::Vector3d & rayTracedBeam) {
        std::cout << rayTracedBeam(0) << " " << rayTracedBeam(1) << " " << rayTracedBeam(2) << std::endl;
    }
    
        
protected:

    /** the svp selection strategy*/
    SvpSelectionStrategy & svpStrategy;

    /**the current surface sound speed*/
    double currentSurfaceSoundSpeed;

    /**Vector of pings*/
    std::vector<Ping> pings;

    /**Vector of positions*/
    std::vector<Position> positions;

    /**vector of attitudes*/
    std::vector<Attitude> attitudes;

    /**Vector of SoundVelocityProfile*/
    std::vector<SoundVelocityProfile*> svps;
    
    /**the distance between transducer and water line*/
    double transducerDraft = 0.0;
};



#endif /* DATAGRAMRAYTRACER_HPP */

