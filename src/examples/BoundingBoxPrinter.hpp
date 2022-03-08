/*
 * Copyright 2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

#ifndef BOUNDINGBOXPRINTER_HPP
#define BOUNDINGBOXPRINTER_HPP

#include "../datagrams/DatagramEventHandler.hpp"

/*!
* \brief Datagram printer class.
* \author Guillaume Labbe-Morissette
*
* Extention of Datagram processor class
*/
class BoundingBoxPrinter : public DatagramEventHandler{
private:

	double minLongitude = (std::numeric_limits<double>::max)();
	double maxLongitude = std::numeric_limits<double>::lowest();

	double minLatitude  = (std::numeric_limits<double>::max)();
	double maxLatitude  = std::numeric_limits<double>::lowest();


public:
	BoundingBoxPrinter(){

	}

	~BoundingBoxPrinter(){

	}
        
        void processSidescanData(SidescanPing * ping) {
            if(ping->getPosition()) {
                processPosition(ping->getTimestamp(),
                    ping->getPosition()->getLongitude(),
                    ping->getPosition()->getLatitude(),
                    ping->getPosition()->getEllipsoidalHeight()
                );
            }
        }

	void processPosition(uint64_t microEpoch,double longitude,double latitude,double height){
		if(longitude < minLongitude){
                    minLongitude = longitude;
		}

		if(longitude > maxLongitude){
                    maxLongitude = longitude;
		}

		if(latitude < minLatitude){
                    minLatitude = latitude;
		}

		if(latitude > maxLatitude){
                    maxLatitude = latitude;
		}
	}

	double getMinimumLongitude(){ return minLongitude;}
	double getMaximumLongitude(){ return maxLongitude;}
	double getMinimumLatitude() { return minLatitude;}
	double getMaximumLatitude(){ return maxLatitude;}
};

#endif /* BOUNDINGBOXPRINTER_HPP */

