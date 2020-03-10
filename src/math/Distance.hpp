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

/*
        double dx, dy, dz;
        latitude1 -= latitude2;
        latitude1 *= D2R, longitude1 *= D2R, longitude2 *= D2R;
    
        dz = sin(longitude1) - sin(longitude2);
        dx = cos(latitude1) * cos(longitude1) - cos(longitude2);
        dy = sin(latitude1) * cos(longitude1);
        return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * 6371000;    
*/



        // From https://en.wikipedia.org/wiki/Haversine_formula

        // What is inside the square root is called "h" on Wikipedia
        // From https://en.wikipedia.org/wiki/Haversine_formula
        // "When using these formulae, one must ensure that h does not exceed 1 due to a 
        // floating point error (d is only real for h from 0 to 1). h only approaches 1 for 
        // antipodal points (on opposite sides of the sphere)—in this region, relatively 
        // large numerical errors tend to arise in the formula when finite precision is used. 
        // Because d is then large (approaching πR, half the circumference) a small error is 
        // often not a major concern in this unusual case (although there are other great-circle 
        // distance formulas that avoid this problem).

        longitude1 *= D2R;
        latitude1 *= D2R;
        longitude2 *= D2R;
        latitude2 *= D2R;

        double sin2LatDiffOver2 = pow( sin( ( latitude2 - latitude1 ) / 2 ), 2 );

        double sin2LongDiffOver2 = pow( sin( ( longitude2 - longitude1 ) / 2 ), 2 );

        double insideSqrt = sin2LatDiffOver2 + cos( latitude1 ) * cos( latitude2 ) * sin2LongDiffOver2;

        if ( insideSqrt > 1.0 )
            std::cerr << "\n\n--- Distance::haversine(): the calculation inside the square root is "
            << insideSqrt << ".\n"
            << "This must be a floating point error as it should be between 0 and 1 inclusively.\n" << std::endl;

        return 2 * 6371000 * asin( sqrt( insideSqrt ) );

    }
};


#endif /* DISTANCE_HPP */

