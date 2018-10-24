/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   SoundVelocityProfileTest.cpp
 * Author: jordan
 *
 */

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <iomanip>
#include "catch.hpp"
#include "../src/SoundVelocityProfile.hpp"
#include "../src/utils/Exception.hpp"

TEST_CASE("Sound velocity profile from original MSPAC") {
    
    SoundVelocityProfile svp;
    bool readSuccess = svp.readFile("test/data/SVP/SVP.txt", 0.7);
    
    REQUIRE(readSuccess);
    
    REQUIRE(svp.getDepth().rows() == 31);
    REQUIRE(svp.getSoundSpeed().rows() == 31);
    
    int n = 31;
    
    Eigen::VectorXd depth(n);
    depth << 0, 0.31, 0.84, 1.34, 1.84, 2.35, 2.86, 3.37,
            3.88, 4.4, 4.9, 5.42, 5.95, 6.45, 6.96, 7.47, 7.99, 8.49,
            9, 9.51, 10.03, 10.53, 11.03, 11.55, 12.05, 12.56, 13.07,
            13.6, 14.12, 14.63, 15.13;
    
    Eigen::VectorXd soundSpeed(n);
    soundSpeed << 1506.5716, 1506.553, 1506.554, 1506.561,
            1506.609, 1506.631, 1506.505, 1506.501, 1506.501, 1506.519,
            1506.511, 1506.534, 1506.517, 1506.568, 1506.532, 1506.513,
            1506.416, 1506.329, 1505.866, 1505.817, 1505.806, 1505.822,
            1505.787, 1505.765, 1505.654, 1505.439, 1505.209, 1505.154,
            1505.156, 1505.136, 1505.048;
    
    double depthPrecision = 0.000000001;
    Eigen::VectorXd testDepth = svp.getDepth().array() - depth.array();
    REQUIRE(testDepth.cwiseAbs().maxCoeff() < depthPrecision);
    
    double speedPrecision = 0.000000001;
    Eigen::VectorXd testSpeed = svp.getSoundSpeed().array() - soundSpeed.array();
    REQUIRE(testSpeed.cwiseAbs().maxCoeff() < speedPrecision);
    
    
    double gradientPrecision = 0.000000001;
    Eigen::VectorXd testCelerityDifferences = soundSpeed.tail(n - 1) - soundSpeed.head(n - 1);
    Eigen::VectorXd testDepthDifferences = depth.tail(n - 1) - depth.head(n - 1);
    Eigen::VectorXd gradient = testCelerityDifferences.cwiseQuotient(testDepthDifferences);
    Eigen::VectorXd testGradient = svp.getGradient().array() - gradient.array();
    REQUIRE(testGradient.cwiseAbs().maxCoeff() < gradientPrecision);
}

