/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   CoordinateTransformTest.cpp
 * Author: glm,jordan
 *
 */

#include "catch.hpp"
#include "../src/math/CoordinateTransform.hpp"
#include <Eigen/Dense>
#include "../src/Position.hpp"

TEST_CASE("Coordinate Transform Test") {

    // prime meridian, equator, on ellipsoid
    Position testPosition(0, 0, 0);

    Eigen::Vector3d testPositionECEF;
    CoordinateTransform::getPositionInTerrestialReferenceFrame(testPositionECEF,testPosition);

    Eigen::Vector3d expectedPositionECEF;
    expectedPositionECEF << 6378137.0 , 0, 0;

    double positionPrecision = 0.00000001;

    REQUIRE( testPositionECEF.isApprox(expectedPositionECEF, positionPrecision) );

    // North Pole on ellipsoid
    Position northPole(90, 0, 0);
    Eigen::Vector3d northPoleECEF;
    CoordinateTransform::getPositionInTerrestialReferenceFrame(northPoleECEF,northPole);

    Eigen::Vector3d expectedNorthPoleECEF;
    expectedNorthPoleECEF << 0, 0, 6356752.3142; // WGS 84 polar semi-minor axis

    double northPolePrecision = 0.00000001;
    REQUIRE( northPoleECEF.isApprox(expectedNorthPoleECEF, northPolePrecision) );
}