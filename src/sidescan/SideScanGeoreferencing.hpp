/*
 * Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   SideScanGeoreferencing.hpp
 * Author: Jordan McManus <jordan.mcmanus@cidco.ca>
 *
 * Created on March 11, 2020, 12:40 PM
 */

#ifndef SIDESCANGEOREFERENCING_HPP
#define SIDESCANGEOREFERENCING_HPP

#include <Eigen/Dense>
#include "../Position.hpp"
#include "../math/CoordinateTransform.hpp"

class SideScanGeoreferencing {
public:

    static void georeferenceSideScanEcef(
            Eigen::Vector3d & antennaEcef,
            Eigen::Vector3d & antenna2TowPointLeverArmEcef,
            Eigen::Vector3d & layBackEcef,
            Eigen::Vector3d & sideDistanceEcef,
            Position & georeferencedPosition
            ) {

        Eigen::Vector3d objectPositionEcef =
                antennaEcef +
                antenna2TowPointLeverArmEcef +
                layBackEcef +
                sideDistanceEcef;

        CoordinateTransform::convertECEFToLongitudeLatitudeElevation(objectPositionEcef, georeferencedPosition);
    }
};

#endif /* SIDESCANGEOREFERENCING_HPP */

