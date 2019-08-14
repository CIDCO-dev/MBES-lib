/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef DATAGRAMPROCESSOR_HPP
#define DATAGRAMPROCESSOR_HPP

#include "../svp/SoundVelocityProfile.hpp"

/*!
* \brief Datagram event handler class
* \author Guillaume Morissette
*
* Provides virtual methods to treat the sonar datagrams' contents
*/
class DatagramEventHandler{
public:

	/**Create an event handler*/
	DatagramEventHandler(){};

	/**Destroy the event handler*/
	virtual ~DatagramEventHandler(){};


	/**
	* Datagrams either use numerical IDs or characters
	* @param id Datagram tag
	*/
	virtual void processDatagramTag(int id){};


	/**
	* Convention for attitude angles (in degrees)
	*
	* @param pitch Positive value is nose up (0 to 360)
	* @param roll Positive value is roll to starboard (right) (0 to 360)
	* @param heading Positive clockwise (magnetic north) (NORMALIZED TO 0 to 360)
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
	* @param microEpoch Timestamp
	* @param id Ping id
	* @param beamAngle NEGATIVE to port (left) side, nadir is 0 degrees, POSITIVE to starboard (right) side
	* @param tiltAngle POSITIVE forward, NEGATIVE backward
	* @param twoWayTravelTime
	* @param quality Quality flag
	* @param intensity Intensity flag
	*/
	virtual void processPing(uint64_t microEpoch,long id, double beamAngle,double tiltAngle,double twoWayTravelTime,uint32_t quality,int32_t intensity){};

	/**
	* Convention for Swath
	*
	* @param surfaceSoundSpeed the swath surface sound speed
	*/
	virtual void processSwathStart(double surfaceSoundSpeed){};

	/**
	* Processes a sound velocity profile, from a SSP profiler or CTD profiler
	* @param svp Sound velocity profile
	*/
	virtual void processSoundVelocityProfile(SoundVelocityProfile * svp){ delete svp;};

        virtual void processSidescanData(unsigned int channel,std::vector<double> & data){}
};


#endif
