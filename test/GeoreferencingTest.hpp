/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   GeoreferencingTest.hpp
 * Author: jordan
 */

#ifndef GEOREFERENCINGTEST_HPP
#define GEOREFERENCINGTEST_HPP


#include "catch.hpp"
#include "../src/georeferencing/Georeferencing.hpp"
#include "../src/math/Boresight.hpp"
#include "../src/utils/Constants.hpp"

TEST_CASE("Georeferencing LGF test") {
    
    Georeferencing * georef = new GeoreferencingLGF();
    
    /*Build attitude*/
    double rollDegrees = 0.02740*R2D; // roll at receive
    double pitchDegrees = 0.02432*R2D; // pitch at transmit
    double headingDegrees = 0.10561*R2D; // heading at transmit
    Attitude attitude(0, rollDegrees, pitchDegrees, headingDegrees);
    
    /*Build Position*/
    double latitudeDegrees = 0.859283*R2D;
    double longitudeDegrees = -1.18908*R2D;
    double ellipsoidalHeight = -25.7532;
    Position position(0, latitudeDegrees, longitudeDegrees, ellipsoidalHeight);
    
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
    
    /*Obtain svp*/
    std::string svpFilePath = "test/data/rayTracingTestData/SVP-0.svp";
    SoundVelocityProfile svp;
    svp.read(svpFilePath);
    
    
    Eigen::Vector3d leverArm;
    leverArm << 0.0, 0.0, 0.0;
    
    /*Build Boresight Matrix*/
    double rollBoresightDegrees=0.62;
    double pitchBoresightDegrees=0.0;
    double headingBoresightDegrees=0.0;
    Attitude boresightAngles(0,rollBoresightDegrees, pitchBoresightDegrees, headingBoresightDegrees);
    Eigen::Matrix3d boresightMatrix;
    Boresight::buildMatrix(boresightMatrix,boresightAngles);
    
    /*Perform georeferencing in LGF*/
    Eigen::Vector3d georeferencedPing;
    georef->georeference(georeferencedPing, attitude, position, ping, svp, leverArm, boresightMatrix);
    
    std::cout << std::endl;
    std::cout << "georeferencedPing" << std::endl;
    std::cout << georeferencedPing << std::endl;
    
    Eigen::Vector3d expectedGeoreferencedPing;
    expectedGeoreferencedPing << -26.88260, 7.35494, 10.47586;
            
    double georefTestTreshold = 1e-2;
    REQUIRE(std::abs(expectedGeoreferencedPing(0) - georeferencedPing(0)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeoreferencedPing(1) - georeferencedPing(1)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeoreferencedPing(2) - georeferencedPing(2)) < georefTestTreshold);
}


#endif /* GEOREFERENCINGTEST_HPP */

