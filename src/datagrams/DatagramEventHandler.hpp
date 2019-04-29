#ifndef DATAGRAMPROCESSOR_HPP
#define DATAGRAMPROCESSOR_HPP

#include "../SoundVelocityProfile.hpp"

/**
 * @author Guillaume Morissette
 * Fournit des methodes virtuelles pour le traitement des contenus des datagrammes de sonar
 */

/*!
 * \brief Datagram event handler class
 */
class DatagramEventHandler{
	public:

                /**Create an event handler*/
		DatagramEventHandler(){};

                /**Destroy the event handler*/
		virtual ~DatagramEventHandler(){};


		/*
                 * Datagrams either use numerical IDs or characters
                 */
		virtual void processDatagramTag(int id){};


		/* Convention for attitude angles (in degrees)
		 * 
		 * Pitch: Positive value is nose up (0 to 360)
		 * Roll: Positive value is roll to starboard (right) (0 to 360)
		 * Heading: gyro (magnetic north) (NORMALIZED TO 0 to 360)
		 */
                virtual void processAttitude(uint64_t microEpoch,double heading,double pitch,double roll){};


                /**
                 * Convention for position
                 * 
                 * @param microEpoch the position timestamp
                 * @param longitude the position longitude
                 * @param latitude the position latitude
                 * @param height the position ellipsoidal Height
                 */
                virtual void processPosition(uint64_t microEpoch,double longitude,double latitude,double height){};


		/**
		* Convention for ping angles (in degrees)
		* NED
		* Beam angle: NEGATIVE to port (left) side, nadir is 0 degrees, POSITIVE to starboard (right) side
		* Tilt angle: POSITIVE forward, NEGATIVE backward
		*/
                virtual void processPing(uint64_t microEpoch,long id, double beamAngle,double tiltAngle,double twoWayTravelTime,uint32_t quality,uint32_t intensity){};

                /**
                 * Convention for Swath
                 * 
                 * @param surfaceSoundSpeed the swath surface sound speed
                 */
                virtual void processSwathStart(double surfaceSoundSpeed){};

		/*
                 * Processes a sound velocity profile, from a SSP profiler or CTD profiler
		 */
		virtual void processSoundVelocityProfile(SoundVelocityProfile * svp){ delete svp;};


};


#endif
