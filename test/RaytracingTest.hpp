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
#include "../src/svp/CarisSvpFile.hpp"

TEST_CASE("Ray tracing test") {
    
    /*Obtain svp*/
    std::string svpFilePath = "test/data/rayTracingTestData/SVP-0.svp";
    CarisSvpFile svps;
    svps.readSvpFile(svpFilePath);
    SoundVelocityProfile * svp = svps.getSvps()[0];

    /*Build Ping*/
    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;
    double surfaceSoundSpeed = 1446.4250488;
    double twoWayTravelTime = 0.0091418369 * 2;
    double alongTrackAngle = 0.0;
    double acrossTrackAngle = 0.7031931281 * R2D;

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
    double rollDegrees = 0.0273983876 * R2D; // roll at receive
    double pitchDegrees = 0.0243184966 * R2D; // pitch at transmit
    double headingDegrees = 0.1056083942 * R2D; // heading at transmit
    Attitude attitude(0, rollDegrees, pitchDegrees, headingDegrees);
    
    Eigen::Matrix3d imu2nav;
    CoordinateTransform::getDCM(imu2nav, attitude);
    
    /* Perform the ray tracing*/
    Eigen::Vector3d ray;
    Raytracing::rayTrace(ray, ping, *svp, boresightMatrix, imu2nav);
    
    /* Test ray tracing values */
    Eigen::Vector3d expectedRay;
    expectedRay << -0.6131269227, 8.2028276296, 10.4345279516;
    
    double rayTestTreshold = 6e-3;
    REQUIRE(std::abs(expectedRay(0) - ray(0)) < rayTestTreshold);
    REQUIRE(std::abs(expectedRay(1) - ray(1)) < rayTestTreshold);
    REQUIRE(std::abs(expectedRay(2) - ray(2)) < rayTestTreshold);
}

#endif /* RAYTRACINGTEST_HPP */

