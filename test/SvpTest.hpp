/*
 * Copyright 2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   SvpTest.hpp
 * Author: Jordan McManus
 */

#ifndef SVPTEST_HPP
#define SVPTEST_HPP

#include "catch.hpp"
#include "../src/svp/SoundVelocityProfile.hpp"

TEST_CASE("SVP test, nul gradient") {
    /*Build an svp with a single constant speed layer*/
    SoundVelocityProfile * svp = new SoundVelocityProfile();

    double depth1 = 0;
    double speed1 = 1500;

    double depth2 = 10;
    double speed2 = 1500;
    
    svp->add(depth1, speed1);
    svp->add(depth2, speed2);
    
    Eigen::VectorXd depths = svp->getDepths();
    Eigen::VectorXd speeds = svp->getSpeeds();
    Eigen::VectorXd gradient = svp->getSoundSpeedGradient();
    
    double eps = 1e-9;
    REQUIRE(std::abs(depths(0) - depth1) < eps);
    REQUIRE(std::abs(depths(1) - depth2) < eps);
    
    REQUIRE(std::abs(speeds(0) - speed1) < eps);
    REQUIRE(std::abs(speeds(1) - speed2) < eps);
    
    REQUIRE(std::abs(gradient(0) - 0) < eps);
    
}

TEST_CASE("SVP index of layer for depth") {
    SoundVelocityProfile * svp = new SoundVelocityProfile();

    double depth1 = 5;
    double speed1 = 1500;

    double depth2 = 10;
    double speed2 = 1500;
    
    svp->add(depth1, speed1);
    svp->add(depth2, speed2);
    
    REQUIRE(svp->getLayerIndexForDepth(2) == 0); // above first sample
    REQUIRE(svp->getLayerIndexForDepth(8) == 1); // between the 2 samples
    REQUIRE(svp->getLayerIndexForDepth(15) == 2); // below last sample
}


TEST_CASE("SVP test, constant gradient") {
    /*Build an svp with a single constant gradient layer*/
    SoundVelocityProfile * svp = new SoundVelocityProfile();

    double depth1 = 0;
    double speed1 = 1400;

    double depth2 = 10;
    double speed2 = 1500;
    
    svp->add(depth1, speed1);
    svp->add(depth2, speed2);
    
    Eigen::VectorXd depths = svp->getDepths();
    Eigen::VectorXd speeds = svp->getSpeeds();
    Eigen::VectorXd gradient = svp->getSoundSpeedGradient();
    
    double eps = 1e-9;
    REQUIRE(std::abs(depths(0) - depth1) < eps);
    REQUIRE(std::abs(depths(1) - depth2) < eps);
    
    REQUIRE(std::abs(speeds(0) - speed1) < eps);
    REQUIRE(std::abs(speeds(1) - speed2) < eps);
    
    REQUIRE(std::abs(gradient(0) - 10) < eps);
    
}

TEST_CASE("get layer index at depth test") {
    /*Build an svp with a 2 constant gradient layers*/
    SoundVelocityProfile * svp = new SoundVelocityProfile();

    double depth1 = 5;
    double speed1 = 1400;

    double depth2 = 10;
    double speed2 = 1500;
    
    double depth3 = 15;
    double speed3 = 1600;
    
    svp->add(depth1, speed1);
    svp->add(depth2, speed2);
    svp->add(depth3, speed3);
    
    Eigen::VectorXd depths = svp->getDepths();
    Eigen::VectorXd speeds = svp->getSpeeds();
    Eigen::VectorXd gradient = svp->getSoundSpeedGradient();
    
    REQUIRE(gradient.size() == 2);
    
    REQUIRE(svp->getLayerIndexForDepth(1) == 0);
    REQUIRE(svp->getLayerIndexForDepth(depth1) == 1);
    REQUIRE(svp->getLayerIndexForDepth(6) == 1);
    REQUIRE(svp->getLayerIndexForDepth(depth2) == 2);
    REQUIRE(svp->getLayerIndexForDepth(11) == 2);
    REQUIRE(svp->getLayerIndexForDepth(depth3) == 3);
    REQUIRE(svp->getLayerIndexForDepth(16) == 3);
    
}

#endif /* SVPTEST_HPP */

