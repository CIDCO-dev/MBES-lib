/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   CoordinateTransformTest.cpp
 * Author: jordan
 *
 */

#include "catch.hpp"
#include "../src/math/CoordinateTransform.hpp"
#include <Eigen/Dense>
#include "../src/Position.hpp"

TEST_CASE("Coordinate Transform Test") {
    
    // prime meridian, equator, on ellipsoid
    Position testPosition(0, 0, 0);
    
    Eigen::Vector3d * testPositionInTRF = CoordinateTransform::getPositionInTerrestialReferenceFrame(testPosition);
    Eigen::Vector3d expectedPositionInTRF;
    expectedPositionInTRF << 6378137.0 , 0, 0;
    
    double positionPrecision = 0.00000001;
    REQUIRE( testPositionInTRF->isApprox(expectedPositionInTRF, positionPrecision) );
    delete testPositionInTRF;
    
    // North Pole on ellipsoid
    Position northPole(90, 0, 0);
    Eigen::Vector3d * northPoleInTRF = CoordinateTransform::getPositionInTerrestialReferenceFrame(northPole);
    
    Eigen::Vector3d expectedNorthPoleInTRF;
    expectedNorthPoleInTRF << 0, 0, 6356752.3142; // WGS 84 polar semi-minor axis
    
    double northPolePrecision = 0.00000001;
    REQUIRE( northPoleInTRF->isApprox(expectedNorthPoleInTRF, northPolePrecision) );
    delete northPoleInTRF;
    
    
    
}