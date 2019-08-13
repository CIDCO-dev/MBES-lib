/*
 * Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   RaytracingTest.hpp
 * Author: jordan
 *
 */

#ifndef RAYTRACINGTEST_HPP
#define RAYTRACINGTEST_HPP

#include "catch.hpp"
#include "../src/georeferencing/Raytracing.hpp"
#include "../src/Ping.hpp"
#include "../src/math/CoordinateTransform.hpp"
#include "../src/math/Boresight.hpp"
#include "../src/utils/Constants.hpp"

TEST_CASE("Ray tracing test") {
    
    /*Obtain svp*/
    std::string svpFilePath = "test/data/rayTracingTestData/SVP-0.svp";
    SoundVelocityProfile svp;
    svp.read(svpFilePath);

    /*Build Ping*/
    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;
    double surfaceSoundSpeed = 1446.42505;
    double twoWayTravelTime = 0.00914*2;
    double alongTrackAngle = 0.0;
    double acrossTrackAngle = 0.70319*R2D;

    Ping ping(
            microEpoch,
            id,
            quality,
            intensity,
            surfaceSoundSpeed,
            twoWayTravelTime,
            alongTrackAngle,
            acrossTrackAngle
            );
    
    /*Obtain Boresight Matrix*/
    double rollBoresightDegrees=0.62;
    double pitchBoresightDegrees=0.0;
    double headingBoresightDegrees=0.0;
    Attitude boresightAngles(0,rollBoresightDegrees, pitchBoresightDegrees, headingBoresightDegrees);
    Eigen::Matrix3d boresightMatrix;
    Boresight::buildMatrix(boresightMatrix,boresightAngles);
    
    /*Obtain imu to nav DCM*/
    double rollDegrees = 0.02740*R2D; // roll at receive
    double pitchDegrees = 0.02432*R2D; // pitch at transmit
    double headingDegrees = 0.10561*R2D; // heading at transmit
    Attitude attitude(0, rollDegrees, pitchDegrees, headingDegrees);
    
    Eigen::Matrix3d imu2nav;
    CoordinateTransform::getDCM(imu2nav, attitude);
    
    /* Perform the ray tracing*/
    Eigen::Vector3d ray;
    Raytracing::rayTrace(ray, ping, svp, boresightMatrix, imu2nav);
    
    /* Test ray tracing values */
    Eigen::Vector3d expectedRay;
    expectedRay << -0.61313, 8.20283, 10.43453;
    
    double rayTestTreshold = 1e-2;
    REQUIRE(std::abs(expectedRay(0) - ray(0)) < rayTestTreshold);
    REQUIRE(std::abs(expectedRay(1) - ray(1)) < rayTestTreshold);
    REQUIRE(std::abs(expectedRay(2) - ray(2)) < rayTestTreshold);
}

#endif /* RAYTRACINGTEST_HPP */

