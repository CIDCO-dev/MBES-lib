/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   GeoreferencingTest.cpp
 * Author: jordan
 *
 * Created on September 14, 2018, 11:44 AM
 */


#include "catch.hpp"
#include "../src/SoundVelocityProfile.hpp"
#include "../src/Ping.hpp"
#include "../src/Swath.hpp"
#include "../src/SurveySystem.hpp"
#include "../src/math/DCM.hpp"
#include "../src/SurveyLine.hpp"
#include "../src/Attitude.hpp"
#include "../src/Georeferencing.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>

TEST_CASE("Georeferencing Test") {

    Eigen::Vector3d launchVectorStraightDown(0, 0, 1);
    Eigen::Vector3d rayStraightDown;

    Georeferencing::constantCelerityRayTracing(rayStraightDown,launchVectorStraightDown, 1500, 1);
    Eigen::Vector3d expectedRayStraightDown(0, 0, 1500);

    double straightDownPrecision = 0.000000001;
    REQUIRE(rayStraightDown.isApprox(expectedRayStraightDown, straightDownPrecision));
}
