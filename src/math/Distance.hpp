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

#define D2R (M_PI / 180)

class Distance{
    static double haversine(double longitude1, double latitude1, double longitude2, double latitude2){

	double dx, dy, dz;
	latitude1 -= latitude2;
	latitude1 *= D2R, longitude1 *= D2R, longitude2 *= D2R;
 
	dz = sin(longitude1) - sin(longitude2);
	dx = cos(latitude1) * cos(longitude1) - cos(longitude2);
	dy = sin(latitude1) * cos(longitude1);
	return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * 6371;    
    }
};


#endif /* DISTANCE_HPP */

