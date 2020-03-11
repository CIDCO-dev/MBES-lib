/*
 * Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   SideScanGeoreferencingTest.hpp
 * Author: Jordan McManus <jordan.mcmanus@cidco.ca>
 *
 * Created on March 11, 2020, 1:18 PM
 */

#ifndef SIDESCANGEOREFERENCINGTEST_HPP
#define SIDESCANGEOREFERENCINGTEST_HPP

#include "catch.hpp"
#include <Eigen/Dense>
#include "../src/Position.hpp"
#include "../src/sidescan/SideScanGeoreferencing.hpp"
#include "../src/math/CoordinateTransform.hpp"
#include "../src/utils/Constants.hpp"

TEST_CASE("Georeferencing Side Scan No LeverArm No Layback test") {

    double earthRadius = 6.3781e6; // in meters
    double objectLatRadians = 0.0;
    double objectLonRadians = 0.0 + 10 / (earthRadius); // 10 meters east of Greenwich
    double objectHeight = 0.0;

    double shipLat = 0.0;
    double shipLon = 0.0;
    double shipHeight = 0.0;
    Position shipPositionAtEquatorGreenwichMeridian(0, shipLat, shipLon, shipHeight);

    Eigen::Vector3d shipPositionEcef;
    CoordinateTransform::getPositionECEF(shipPositionEcef, shipPositionAtEquatorGreenwichMeridian);

    Eigen::Vector3d sideDistanceEcef(0, 10, 0);

    Eigen::Vector3d antenna2TowPointLeverArmEcef(0, 0, 0);
    Eigen::Vector3d laybackEcef(0, 0, 0);

    Position objectPosition(0, 0.0, 0.0, 0.0);
    SideScanGeoreferencing::georeferenceSideScanEcef(
            shipPositionEcef,
            antenna2TowPointLeverArmEcef,
            laybackEcef,
            sideDistanceEcef,
            objectPosition);

    double latLonTreshold = 1e-9; // about 5.7e-8 degrees
    double heightTrashold = 1e-5; // 10 micrometers
    REQUIRE(std::abs(objectPosition.getLatitude() * D2R - objectLatRadians) < latLonTreshold);
    REQUIRE(std::abs(objectPosition.getLongitude() * D2R - objectLonRadians) < latLonTreshold);
    REQUIRE(std::abs(objectPosition.getEllipsoidalHeight() - objectHeight) < heightTrashold);
}

#endif /* SIDESCANGEOREFERENCINGTEST_HPP */

