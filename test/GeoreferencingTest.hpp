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

    GeoreferencingLGF * georef = new GeoreferencingLGF();

    /*Build centroid position*/
    double latitudeCentroidDegrees = 0.859286627204 * R2D;
    double longitudeCentroidDegrees = -1.189078930041 * R2D;
    double ellipsoidalCentroidHeight = -25.711914675768;
    Position positionCentroid(0, latitudeCentroidDegrees, longitudeCentroidDegrees, ellipsoidalCentroidHeight);

    georef->setCentroid(positionCentroid);


    /*Build attitude*/
    double rollDegrees = 0.0273983876 * R2D; // roll at receive
    double pitchDegrees = 0.0243184966 * R2D; // pitch at transmit
    double headingDegrees = 0.1056083942 * R2D; // heading at transmit
    Attitude attitude(0, rollDegrees, pitchDegrees, headingDegrees);

    /*Build Position*/
    double latitudeDegrees = 0.859282504615 * R2D;
    double longitudeDegrees = -1.189079133235 * R2D;
    double ellipsoidalHeight = -25.753195024025;
    Position position(0, latitudeDegrees, longitudeDegrees, ellipsoidalHeight);

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

    /*Obtain svp*/
    std::string svpFilePath = "test/data/rayTracingTestData/SVP-0.svp";
    SoundVelocityProfile svp;
    svp.read(svpFilePath);


    Eigen::Vector3d leverArm;
    leverArm << 0.0, 0.0, 0.0;

    /*Build Boresight Matrix*/
    double rollBoresightDegrees = 0.62;
    double pitchBoresightDegrees = 0.0;
    double headingBoresightDegrees = 0.0;
    Attitude boresightAngles(0, rollBoresightDegrees, pitchBoresightDegrees, headingBoresightDegrees);
    Eigen::Matrix3d boresightMatrix;
    Boresight::buildMatrix(boresightMatrix, boresightAngles);

    /*Perform georeferencing in LGF*/
    Eigen::Vector3d georeferencedPing;
    georef->georeference(georeferencedPing, attitude, position, ping, svp, leverArm, boresightMatrix);

    Eigen::Vector3d expectedGeoreferencedPing;
    expectedGeoreferencedPing << -26.8825997032, 7.3549385469, 10.4758625062;

    double georefTestTreshold = 6e-3;
    REQUIRE(std::abs(expectedGeoreferencedPing(0) - georeferencedPing(0)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeoreferencedPing(1) - georeferencedPing(1)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeoreferencedPing(2) - georeferencedPing(2)) < georefTestTreshold);
}


#endif /* GEOREFERENCINGTEST_HPP */

