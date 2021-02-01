/*
 * Copyright 2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   VerticalHorizontalRayTracingBiais.hpp
 * Author: Jordan McManus
 */

#ifndef VERTICALHORIZONTALRAYTRACINGBIAIS_HPP
#define VERTICALHORIZONTALRAYTRACINGBIAIS_HPP



#include "catch.hpp"

#include <string>
#include <vector>
#include "../src/Attitude.hpp"
#include "../src/Ping.hpp"
#include "../src/svp/SoundVelocityProfile.hpp"
#include "../src/svp/CarisSvpFile.hpp"
#include "../src/math/Boresight.hpp"
#include "../src/math/CoordinateTransform.hpp"
#include "../src/georeferencing/Raytracing.hpp"


TEST_CASE("Vertical and Horizontal bias") {
    
    std::string svp1_filename = "test/data/SVP/svpJeanGuy/s20200713_071750.svp";
    std::string svp2_filename = "test/data/SVP/svpJeanGuy/s20200713_072911.svp";
    
    CarisSvpFile carisSvp1;
    bool svp1Read = carisSvp1.readSvpFile(svp1_filename);
    SoundVelocityProfile * svp1 = carisSvp1.getSvps()[0];
    
    
    CarisSvpFile carisSvp2;
    bool svp2Read = carisSvp2.readSvpFile(svp2_filename);
    SoundVelocityProfile * svp2 = carisSvp2.getSvps()[0];
    
    REQUIRE(svp1Read);
    REQUIRE(svp2Read);
    
    // Attitude
    double rollDegrees = 0.0273983876 * R2D; // roll at receive
    double pitchDegrees = 0.0243184966 * R2D; // pitch at transmit
    double headingDegrees = 0.1056083942 * R2D; // heading at transmit
    Attitude attitude(0, rollDegrees, pitchDegrees, headingDegrees);
    Eigen::Matrix3d imu2ned;
    CoordinateTransform::getDCM(imu2ned,attitude);
    
    
    //Boresight
    double rollBoresightDegrees = 0.0;
    double pitchBoresightDegrees = 0.0;
    double headingBoresightDegrees = 0.0;
    Attitude boresightAngles(0, rollBoresightDegrees, pitchBoresightDegrees, headingBoresightDegrees);
    Eigen::Matrix3d boresightMatrix;
    Boresight::buildMatrix(boresightMatrix, boresightAngles);
    
    
    // Ping
    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;
    double surfaceSoundSpeed = 1485.4;
    double oneWayTravelTime = 0.004;
    double twoWayTravelTime = 2*oneWayTravelTime;
    double alongTrackAngle = 0.0;
    double acrossTrackAngle = 45.0; //45 degrees
    double transducerDepth = 4.3;

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

    ping.setTransducerDepth(transducerDepth);
    
    std::vector<Eigen::Vector2d> layerRaysWithSvp1;
    std::vector<double> layerTravelTimesWithSvp1;
    
    std::vector<Eigen::Vector2d> layerRaysWithSvp2;
    std::vector<double> layerTravelTimesWithSvp2;
    
    Eigen::Vector2d rayWithSvp1;
    Eigen::Vector2d rayWithSvp2;
    
    
    Raytracing::planarRayTrace(rayWithSvp1,
            layerRaysWithSvp1,
            layerTravelTimesWithSvp1,
            ping,
            *svp1,
            boresightMatrix,
            imu2ned);
    
    Raytracing::planarRayTrace(rayWithSvp2,
            layerRaysWithSvp2,
            layerTravelTimesWithSvp2,
            ping,
            *svp2,
            boresightMatrix,
            imu2ned);
    
    //TODO: is 1cm constraint strong enough?
    double horizontalBiasEpsilon = 0.01; // 1cm
    double verticalBiasEpsilon = 0.01; // 1cm
    
    Eigen::Vector2d biasSVP = rayWithSvp1-rayWithSvp2;
    REQUIRE(std::abs(biasSVP(0)) < horizontalBiasEpsilon);
    REQUIRE(std::abs(biasSVP(1)) < verticalBiasEpsilon);
}

#endif /* VERTICALHORIZONTALRAYTRACINGBIAIS_HPP */

