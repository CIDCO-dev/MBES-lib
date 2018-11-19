/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   DcmTest.cpp
 * Author: glm,jordan
 *
 */

#include "catch.hpp"
#include "../src/math/DCM.hpp"
#include <Eigen/Dense>
#include "../src/Attitude.hpp"
#include <cmath>

TEST_CASE("Direction Cosine Matrix Test") {

    Eigen::Vector3d testVector = Eigen::Vector3d::Ones();

    Eigen::Matrix3d dcmIdentity;
    DCM::getDcm(dcmIdentity,0.0, 1.0, 0.0, 1.0, 0.0, 1.0);
    REQUIRE(dcmIdentity.isIdentity());

    Attitude attitudeZero(0, 0, 0);
    Eigen::Matrix3d dcmAttitudeZero;
    DCM::getDcm(dcmAttitudeZero,attitudeZero);
    REQUIRE(dcmAttitudeZero.isIdentity());

    // roll 180 degrees
    Attitude attitudeRoll(180, 0, 0);
    Eigen::Matrix3d dcmFromAttitudeRollTest;
    DCM::getDcm(dcmFromAttitudeRollTest,attitudeRoll);
    Eigen::Vector3d testRollVector = dcmFromAttitudeRollTest * testVector;

    double testRollPrecision = 0.000000001;
    REQUIRE(abs(testRollVector(0) - 1.0) < testRollPrecision);
    REQUIRE(abs(testRollVector(1) - -1.0) < testRollPrecision);
    REQUIRE(abs(testRollVector(2) - -1.0) < testRollPrecision);

    // pitch 180 degrees
    Attitude attitudePitch(0, 180, 0);
    Eigen::Matrix3d dcmFromAttitudePitchTest;
    DCM::getDcm(dcmFromAttitudePitchTest,attitudePitch);
    Eigen::Vector3d testPitchVector = dcmFromAttitudePitchTest * testVector;

    double testPitchPrecision = 0.000000001;
    REQUIRE(abs(testPitchVector(0) - -1.0) < testPitchPrecision);
    REQUIRE(abs(testPitchVector(1) - 1.0) < testPitchPrecision);
    REQUIRE(abs(testPitchVector(2) - -1.0) < testPitchPrecision);

    // heading 180 degrees
    Attitude attitudeHeading(0, 0, 180);
    Eigen::Matrix3d dcmFromAttitudeHeadingTest;
    DCM::getDcm(dcmFromAttitudeHeadingTest,attitudeHeading);
    Eigen::Vector3d testHeadingVector = dcmFromAttitudeHeadingTest * testVector;

    double testHeadingPrecision = 0.000000001;
    REQUIRE(abs(testHeadingVector(0) - -1.0) < testHeadingPrecision);
    REQUIRE(abs(testHeadingVector(1) - -1.0) < testHeadingPrecision);
    REQUIRE(abs(testHeadingVector(2) - 1.0) < testHeadingPrecision);

    // random attitude
    double roll = 24.357;
    double pitch = 36.54;
    double heading = 157.43;
    Attitude attitudeRandom(roll, pitch, heading);
    Eigen::Matrix3d dcmFromAttitudeRandomTest;
    DCM::getDcm(dcmFromAttitudeRandomTest,attitudeRandom);

    Eigen::Matrix3d dcmFromEulerAngles;
    dcmFromEulerAngles = Eigen::AngleAxisd(heading*D2R, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch*D2R, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll*D2R, Eigen::Vector3d::UnitX());

    double randomPrecision = 0.000000001;
    Eigen::Matrix3d dcmDifferences = dcmFromAttitudeRandomTest - dcmFromEulerAngles;
    dcmDifferences = dcmDifferences.cwiseAbs();
    REQUIRE(dcmDifferences.maxCoeff() < randomPrecision);
};
