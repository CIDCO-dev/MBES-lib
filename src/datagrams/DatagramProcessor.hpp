#ifndef DATAGRAMPROCESSOR_HPP
#define DATAGRAMPROCESSOR_HPP

#include "../SoundVelocityProfile.hpp"

/**
 * @author Guillaume Morissette
 * Fournit des methodes virtuelles pour le traitement des contenus des datagrammes de sonar
 */


class DatagramProcessor{
	public:
		DatagramProcessor(){};
		virtual ~DatagramProcessor(){};

		/* Convention for attitude angles
		 *
		 * Pitch: Positive value is nose up
		 * Roll: Positive value is roll to starboard (right)
		 * Heading: gyro (magnetic north)
		 */
                virtual void processAttitude(uint64_t microEpoch,double heading,double pitch,double roll){};


                virtual void processPosition(uint64_t microEpoch,double longitude,double latitude,double height){};


		/**
		* Convention for ping angles
		* NED
		* Beam angle: NEGATIVE to port (left) side, nadir is 0 degrees, POSITIVE to starboard (right) side
		* Tilt angle: POSITIVE forward, NEGATIVE backward
		*/
                virtual void processPing(uint64_t microEpoch,long id, double beamAngle,double tiltAngle,double twoWayTravelTime,uint32_t quality,uint32_t intensity){};

                virtual void processSwathStart(double surfaceSoundSpeed){};

		/*
                 * Processes a sound velocity profile, from a SSP profiler or CTD profiler
		 */
		virtual void processSoundVelocityProfile(SoundVelocityProfile * svp){};
};


#endif
