/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/* 
 * File:   Distance.hpp
 * Author: glm
 *
 * Created on September 13, 2019, 3:49 PM
 */

#ifndef DISTANCE_HPP
#define DISTANCE_HPP

#include "../utils/Constants.hpp"

class Distance{
public:
    
    /**
     * Gives the distance in meters between two points on a spheroid 
     * @param longitude1 Point1's longitude
     * @param latitude1 Point1's latitude
     * @param longitude2 Point2's longitude
     * @param latitude2 Point2's latitude
     * @return the distance in meters
     */
    static double haversine(double longitude1, double latitude1, double longitude2, double latitude2){

	double dx, dy, dz;
	latitude1 -= latitude2;
	latitude1 *= D2R, longitude1 *= D2R, longitude2 *= D2R;
 
	dz = sin(longitude1) - sin(longitude2);
	dx = cos(latitude1) * cos(longitude1) - cos(longitude2);
	dy = sin(latitude1) * cos(longitude1);
	return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * 6371000;    
    }
};


#endif /* DISTANCE_HPP */

