/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   DcmTest.cpp
 * Author: jordan
 *
 */

#include "catch.hpp"
#include "../src/math/DCM.hpp"
#include <Eigen/Dense>
#include "../src/Attitude.hpp"
#include <cmath>
#include <random>

TEST_CASE("Direction Cosine Matrix Test") {

    Eigen::Vector3d testVector = Eigen::Vector3d::Ones();

    Eigen::Matrix3d* dcmFromSineCosineTest = DCM::getDcm(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);
    REQUIRE(dcmFromSineCosineTest->isIdentity());
    delete dcmFromSineCosineTest;

    const Attitude attitudeZero(0, 0, 0);
    Eigen::Matrix3d* dcmFromAttitudeTest = DCM::getDcm(attitudeZero);
    REQUIRE(dcmFromAttitudeTest->isIdentity());
    delete dcmFromAttitudeTest;

    // roll 180 degrees
    const Attitude attitudeRoll(180, 0, 0);
    Eigen::Matrix3d* dcmFromAttitudeRollTest = DCM::getDcm(attitudeRoll);
    Eigen::Vector3d testRollVector = (*dcmFromAttitudeRollTest) * testVector;
    delete dcmFromAttitudeRollTest;

    double testRollPrecision = 0.000000001;
    REQUIRE(abs(testRollVector(0) - 1.0) < testRollPrecision);
    REQUIRE(abs(testRollVector(1) - -1.0) < testRollPrecision);
    REQUIRE(abs(testRollVector(2) - -1.0) < testRollPrecision);

    // pitch 180 degrees
    const Attitude attitudePitch(0, 180, 0);
    Eigen::Matrix3d* dcmFromAttitudePitchTest = DCM::getDcm(attitudePitch);
    Eigen::Vector3d testPitchVector = (*dcmFromAttitudePitchTest) * testVector;
    delete dcmFromAttitudePitchTest;

    double testPitchPrecision = 0.000000001;
    REQUIRE(abs(testPitchVector(0) - -1.0) < testPitchPrecision);
    REQUIRE(abs(testPitchVector(1) - 1.0) < testPitchPrecision);
    REQUIRE(abs(testPitchVector(2) - -1.0) < testPitchPrecision);

    // heading 180 degrees
    const Attitude attitudeHeading(0, 0, 180);
    Eigen::Matrix3d* dcmFromAttitudeHeadingTest = DCM::getDcm(attitudeHeading);
    Eigen::Vector3d testHeadingVector = (*dcmFromAttitudeHeadingTest) * testVector;
    delete dcmFromAttitudeHeadingTest;

    double testHeadingPrecision = 0.000000001;
    REQUIRE(abs(testHeadingVector(0) - -1.0) < testHeadingPrecision);
    REQUIRE(abs(testHeadingVector(1) - -1.0) < testHeadingPrecision);
    REQUIRE(abs(testHeadingVector(2) - 1.0) < testHeadingPrecision);

    // random attitude
    std::srand((unsigned int) time(0));
    Eigen::Vector3d randomVect = Eigen::Vector3d::Random();
    double roll = randomVect(0) * 180;
    double pitch = randomVect(1) * 180;
    double heading = abs(randomVect(2))*2 * 360;
    const Attitude attitudeRandom(roll, pitch, heading);
    Eigen::Matrix3d* dcmFromAttitudeRandomTest = DCM::getDcm(attitudeRandom);

    Eigen::Matrix3d dcmFromEulerAngles;
    dcmFromEulerAngles = Eigen::AngleAxisd(heading*D2R, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch*D2R, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll*D2R, Eigen::Vector3d::UnitX());
    
    double randomPrecision = 0.000000001;
    Eigen::Matrix3d dcmDifferences = *dcmFromAttitudeRandomTest - dcmFromEulerAngles;
    dcmDifferences = dcmDifferences.cwiseAbs();
    REQUIRE(dcmDifferences.maxCoeff() < randomPrecision);
    delete dcmFromAttitudeRandomTest;
}