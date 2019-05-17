/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

 /*
 * \author Christian Bouchard
 */

#ifndef HULLOVERLAPHIGHLEVEL_HPP
#define HULLOVERLAPHIGHLEVEL_HPP


#include <cstdint>

#include <utility>      // std::pair, std::make_pair

#include <pcl/common/common_headers.h>

#include <pcl/point_types.h>

#include "HullOverlap.hpp"


class HullOverlapHighLevel
{

public:    
    HullOverlapHighLevel( pcl::PointCloud<pcl::PointXYZ>::ConstPtr line1In, 
                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr line2In,
                    double a, double b, double c, double d,
                    double alphaLine1 = 1.0, double alphaLine2 = 1.0 )

                    : hullOverlap( line1In, line2In, a, b, c, d, alphaLine1, alphaLine2 )
    {       

    }


    std::pair< uint64_t, uint64_t > computePointsInBothHulls( pcl::PointCloud<pcl::PointXYZ>::Ptr line1InBothHull,
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr line2InBothHull )
    {
        return hullOverlap.computeHullsAndPointsInBothHulls( line1InBothHull, line2InBothHull, true );
    }


private:

    HullOverlap hullOverlap;

};

#endif