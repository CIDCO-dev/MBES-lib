/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   GeoreferencingTest.hpp
 * Author: glm, jordan
 */

#ifndef GEOREFERENCINGTEST_HPP
#define GEOREFERENCINGTEST_HPP


#include "catch.hpp"
#include <Eigen/Dense>
#include "../src/Position.hpp"
#include "../src/georeferencing/Georeferencing.hpp"
#include "../src/math/Boresight.hpp"
#include "../src/math/CoordinateTransform.hpp"
#include "../src/utils/Constants.hpp"
#include "../src/svp/SoundVelocityProfileFactory.hpp"
#include "../src/svp/CarisSvpFile.hpp"

#define POSITION_PRECISION 0.00000001



TEST_CASE("Georeferencing LGF test") {

    GeoreferencingLGF georef;

    /*Build centroid position*/
    double latitudeCentroidDegrees = 0.859286627204 * R2D;
    double longitudeCentroidDegrees = -1.189078930041 * R2D;
    double ellipsoidalCentroidHeight = -25.711914675768;
    Position positionCentroid(0, latitudeCentroidDegrees, longitudeCentroidDegrees, ellipsoidalCentroidHeight);

    georef.setCentroid(positionCentroid);


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
    CarisSvpFile svps;
    svps.readSvpFile(svpFilePath);
    SoundVelocityProfile * svp = svps.getSvps()[0];


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
    georef.georeference(georeferencedPing, attitude, position, ping, *svp, leverArm, boresightMatrix);

    Eigen::Vector3d expectedGeoreferencedPing;
    expectedGeoreferencedPing << -26.8825997032, 7.3549385469, 10.4758625062;

    double georefTestTreshold = 2e-2; // 2cm
    REQUIRE(std::abs(expectedGeoreferencedPing(0) - georeferencedPing(0)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeoreferencedPing(1) - georeferencedPing(1)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeoreferencedPing(2) - georeferencedPing(2)) < georefTestTreshold);
}

TEST_CASE("Georeference TRF with position and downward ping only") {
    Eigen::Vector3d georefedPing;

    Attitude attitude(0, 0, 0, 0);
    Position position(0, 48.4525, -68.5232, 15.401);
    Ping ping(0, 0, 0, 0, 0, 0.01, 0.0, 0.0);
    SoundVelocityProfile * svp = SoundVelocityProfileFactory::buildFreshWaterModel();
    ping.setSurfaceSoundSpeed(svp->getSpeeds()(0)); //important now that raytracing uses it
    Eigen::Vector3d leverArm(0, 0, 0);
    Eigen::Matrix3d boresight = Eigen::Matrix3d::Identity();

    GeoreferencingTRF geo;
    geo.georeference(georefedPing, attitude, position, ping, *svp, leverArm, boresight);

    //std::cerr << "GEOREF TRF: " << georefedPing << std::endl;

    Position georefPosition(0, 0, 0, 0);

    CoordinateTransform::convertECEFToLongitudeLatitudeElevation(georefedPing, georefPosition);

    //std::cerr << "Final position: " << std::endl << georefPosition << std::endl << std::endl;

    REQUIRE(abs(georefPosition.getLongitude() - position.getLongitude()) < POSITION_PRECISION);
    REQUIRE(abs(georefPosition.getLatitude() - position.getLatitude()) < POSITION_PRECISION);
    REQUIRE(abs(georefPosition.getEllipsoidalHeight() - (position.getEllipsoidalHeight() - 7.4)) < POSITION_PRECISION);
    
    delete svp;
}

TEST_CASE("Georeference LGF with position and downward ping only") {

    Eigen::Vector3d georefedPing;

    Attitude attitude(0, 0, 0, 0);
    Position position(0, 48.4525, -68.5232, 15.401);
    Ping ping(0, 0, 0, 0, 0, 0.01, 0, 0);
    SoundVelocityProfile * svp = SoundVelocityProfileFactory::buildFreshWaterModel();
    ping.setSurfaceSoundSpeed(svp->getSpeeds()(0)); //important now that raytracing uses it
    Eigen::Vector3d leverArm(0, 0, 0);
    Eigen::Matrix3d boresight = Eigen::Matrix3d::Identity();
    
    /*georef LGF*/
    /*Build Centroid Position*/
    double centroidLatitude = 48.4525;
    double centroidLongitude = -68.5232;
    double centroidEllipsoidHeight = 15.401;
    Position centroidPosition(0, centroidLatitude, centroidLongitude, centroidEllipsoidHeight);
    
    GeoreferencingLGF geo;
    geo.setCentroid(centroidPosition);
    
    geo.georeference(georefedPing, attitude, position, ping, *svp, leverArm, boresight);

    //std::cerr << "GEOREF LGF: " << std::endl << georefedPing << std::endl << std::endl;

    Eigen::Vector3d expectedPosition(0, 0, 7.4);
    REQUIRE(georefedPing.isApprox(expectedPosition, POSITION_PRECISION));
    
    delete svp;
}

TEST_CASE("Georeference TRF with position and perpendicular unit vector ping and non-zero attitude") {
    
    /*Build Centroid Position*/
    double centroidLatitude = 48.4525;
    double centroidLongitude = -68.5232;
    double centroidEllipsoidHeight = 15.401;
    Position centroidPosition(0, centroidLatitude, centroidLongitude, centroidEllipsoidHeight);

    /*Build attitude*/
    double roll = 1.0;
    double pitch = 2.0;
    double heading = 45.0;
    Attitude attitude(0, roll, pitch, heading);

    /*Build Position*/
    double latitude = 48.4525;
    double longitude = -68.5232;
    double ellipsoidHeight = 15.401;
    Position position(0, latitude, longitude, ellipsoidHeight);

    /*Build Ping*/
    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;
    double surfaceSoundSpeed = 0;
    double twoWayTravelTime = 0.01;
    double alongTrackAngle = 0.0;
    double acrossTrackAngle = 0.0;
    
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

    /*Build SVP*/
    SoundVelocityProfile * svp = SoundVelocityProfileFactory::buildFreshWaterModel();
    ping.setSurfaceSoundSpeed(svp->getSpeeds()(0)); //important now that raytracing uses it

    /*Build lever arm*/
    Eigen::Vector3d leverArm(0, 0, 0);

    /*Build boresight matrix*/
    Eigen::Matrix3d boresight = Eigen::Matrix3d::Identity();
    
    /*georef TRF*/
    GeoreferencingTRF geo;
    
    Eigen::Vector3d georefedPingTRF;
    geo.georeference(georefedPingTRF, attitude, position, ping, *svp, leverArm, boresight);
    
    /*georef LGF*/
    GeoreferencingLGF geoLGF;
    geoLGF.setCentroid(centroidPosition);
    
    Eigen::Vector3d georefedPingLGF;
    geoLGF.georeference(georefedPingLGF, attitude, position, ping, *svp, leverArm, boresight);
    
    /*Test georefTRF*/
    Eigen::Vector3d centroidTRF;
    CoordinateTransform::getPositionECEF(centroidTRF, centroidPosition);
    
    Eigen::Matrix3d ned2ecef;
    CoordinateTransform::ned2ecef(ned2ecef,centroidPosition);
    
    Eigen::Vector3d expectedGeorefedPingTRF = centroidTRF + ned2ecef*georefedPingLGF;
    
    double georefTestTreshold = 1e-5;
    REQUIRE(std::abs(expectedGeorefedPingTRF(0) - georefedPingTRF(0)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeorefedPingTRF(1) - georefedPingTRF(1)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeorefedPingTRF(2) - georefedPingTRF(2)) < georefTestTreshold);
    
    delete svp;
}

TEST_CASE("Georeference LGF with position and perpendicular unit vector ping and non-zero attitude"){
    /*Build Centroid Position*/
    double centroidLatitude = 48.4525;
    double centroidLongitude = -68.5232;
    double centroidEllipsoidHeight = 15.401;
    Position centroidPosition(0, centroidLatitude, centroidLongitude, centroidEllipsoidHeight);
    
    /*Build attitude*/
    double roll = 1.0; // 0.017453293
    double pitch = 2.0; // 0.034906585
    double heading = 45.0; // 0.785398185
    Attitude attitude(0, roll, pitch, heading);

    /*Build Position*/
    double latitude = 48.4525;
    double longitude = -68.5232;
    double ellipsoidHeight = 15.401;
    Position position(0, latitude, longitude, ellipsoidHeight);

    /*Build Ping*/
    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;
    double surfaceSoundSpeed = 0;
    double twoWayTravelTime = 0.01;
    double alongTrackAngle = 0.0;
    double acrossTrackAngle = 0.0;
    
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

    /*Build SVP*/
    SoundVelocityProfile * svp = SoundVelocityProfileFactory::buildFreshWaterModel();
    ping.setSurfaceSoundSpeed(svp->getSpeeds()(0)); //important now that raytracing uses it

    /*Build lever arm*/
    Eigen::Vector3d leverArm(0, 0, 0);

    /*Build boresight matrix*/
    Eigen::Matrix3d boresight = Eigen::Matrix3d::Identity();
    
    /*georef LGF*/
    GeoreferencingLGF geo;
    geo.setCentroid(centroidPosition);
    
    Eigen::Vector3d georefedPing;
    geo.georeference(georefedPing, attitude, position, ping, *svp, leverArm, boresight);
    
    Eigen::Vector3d expectedGeorefedPing;
    expectedGeorefedPing << 0.2739082412, 0.0912656601, 7.3943657507;
    
    double georefTestTreshold = 1e-5;
    REQUIRE(std::abs(expectedGeorefedPing(0) - georefedPing(0)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeorefedPing(1) - georefedPing(1)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeorefedPing(2) - georefedPing(2)) < georefTestTreshold);
    
    delete svp;
}

TEST_CASE("Georeference TRF with position and perpendicular unit vector ping and non-zero attitude and non-zero lever-arm"){
    
    /*Build Centroid Position*/
    double centroidLatitude = 48.4525;
    double centroidLongitude = -68.5232;
    double centroidEllipsoidHeight = 15.401;
    Position centroidPosition(0, centroidLatitude, centroidLongitude, centroidEllipsoidHeight);
    
    /*Build attitude*/
    double roll = 1.0; // 0.017453293
    double pitch = 2.0; // 0.034906585
    double heading = 45.0; // 0.785398185
    Attitude attitude(0, roll, pitch, heading);

    /*Build Position*/
    double latitude = 48.4525;
    double longitude = -68.5232;
    double ellipsoidHeight = 15.401;
    Position position(0, latitude, longitude, ellipsoidHeight);

    /*Build Ping*/
    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;
    double surfaceSoundSpeed = 0;
    double twoWayTravelTime = 0.01;
    double alongTrackAngle = 0.0;
    double acrossTrackAngle = 0.0;
    
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

    /*Build SVP*/
    SoundVelocityProfile * svp = SoundVelocityProfileFactory::buildFreshWaterModel();
    ping.setSurfaceSoundSpeed(svp->getSpeeds()(0)); //important now that raytracing uses it

    /*Build lever arm*/
    Eigen::Vector3d leverArm(1, 2, 3);

    /*Build boresight matrix*/
    Eigen::Matrix3d boresight = Eigen::Matrix3d::Identity();
    
    /*georef TRF*/
    GeoreferencingTRF geo;
    
    Eigen::Vector3d georefedPingTRF;
    geo.georeference(georefedPingTRF, attitude, position, ping, *svp, leverArm, boresight);
    
    /*georef LGF*/
    GeoreferencingLGF geoLGF;
    geoLGF.setCentroid(centroidPosition);
    
    Eigen::Vector3d georefedPingLGF;
    geoLGF.georeference(georefedPingLGF, attitude, position, ping, *svp, leverArm, boresight);
    
    /*Test georefTRF*/
    Eigen::Vector3d centroidTRF;
    CoordinateTransform::getPositionECEF(centroidTRF, centroidPosition);
    
    Eigen::Matrix3d ned2ecef;
    CoordinateTransform::ned2ecef(ned2ecef,centroidPosition);
    
    Eigen::Vector3d expectedGeorefedPingTRF = centroidTRF + ned2ecef*georefedPingLGF;
    
    double georefTestTreshold = 1e-5;
    REQUIRE(std::abs(expectedGeorefedPingTRF(0) - georefedPingTRF(0)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeorefedPingTRF(1) - georefedPingTRF(1)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeorefedPingTRF(2) - georefedPingTRF(2)) < georefTestTreshold);
    
    delete svp;
}

TEST_CASE("Georeference LGF with position and perpendicular unit vector ping and non-zero attitude and non-zero lever arm"){
    
    /*Build Centroid Position*/
    double centroidLatitude = 48.4525;
    double centroidLongitude = -68.5232;
    double centroidEllipsoidHeight = 15.401;
    Position centroidPosition(0, centroidLatitude, centroidLongitude, centroidEllipsoidHeight);
    
    /*Build attitude*/
    double roll = 1.0; // 0.017453293
    double pitch = 2.0; // 0.034906585
    double heading = 45.0; // 0.785398185
    Attitude attitude(0, roll, pitch, heading);

    /*Build Position*/
    double latitude = 48.4525;
    double longitude = -68.5232;
    double ellipsoidHeight = 15.401;
    Position position(0, latitude, longitude, ellipsoidHeight);

    /*Build Ping*/
    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;
    double surfaceSoundSpeed = 0;
    double twoWayTravelTime = 0.01;
    double alongTrackAngle = 0.0;
    double acrossTrackAngle = 0.0;
    
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

    /*Build SVP*/
    SoundVelocityProfile * svp = SoundVelocityProfileFactory::buildFreshWaterModel();
    ping.setSurfaceSoundSpeed(svp->getSpeeds()(0)); //important now that raytracing uses it

    /*Build lever arm*/
    Eigen::Vector3d leverArm(1, 2, 3);

    /*Build boresight matrix*/
    Eigen::Matrix3d boresight = Eigen::Matrix3d::Identity();
    
    /*georef LGF*/
    GeoreferencingLGF geo;
    geo.setCentroid(centroidPosition);
    
    Eigen::Vector3d georefedPing;
    geo.georeference(georefedPing, attitude, position, ping, *svp, leverArm, boresight);
    
    Eigen::Vector3d expectedGeorefedPing;
    expectedGeorefedPing << -0.3215086477, 2.2498008231, 10.3920656486;
    
    
    double georefTestTreshold = 1e-5;
    REQUIRE(std::abs(expectedGeorefedPing(0) - georefedPing(0)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeorefedPing(1) - georefedPing(1)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeorefedPing(2) - georefedPing(2)) < georefTestTreshold);
    
    delete svp;
}

TEST_CASE("Georeference TRF with position and perpendicular unit vector ping and non-zero attitude and non-zero lever-arm and non-zero boresight"){
    /*Build Centroid Position*/
    double centroidLatitude = 48.4525;
    double centroidLongitude = -68.5232;
    double centroidEllipsoidHeight = 15.401;
    Position centroidPosition(0, centroidLatitude, centroidLongitude, centroidEllipsoidHeight);
    
    /*Build attitude*/
    double roll = 1.0; // 0.017453293
    double pitch = 2.0; // 0.034906585
    double heading = 45.0; // 0.785398185
    Attitude attitude(0, roll, pitch, heading);

    /*Build Position*/
    double latitude = 48.4525;
    double longitude = -68.5232;
    double ellipsoidHeight = 15.401;
    Position position(0, latitude, longitude, ellipsoidHeight);

    /*Build Ping*/
    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;
    double surfaceSoundSpeed = 0;
    double twoWayTravelTime = 0.01;
    double alongTrackAngle = 0.0;
    double acrossTrackAngle = 0.0;
    
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

    /*Build SVP*/
    SoundVelocityProfile * svp = SoundVelocityProfileFactory::buildFreshWaterModel();
    ping.setSurfaceSoundSpeed(svp->getSpeeds()(0)); //important now that raytracing uses it

    /*Build lever arm*/
    Eigen::Vector3d leverArm(1, 2, 3);

    /*Build boresight matrix*/
    double rollBoresightDegrees = 0.4;
    double pitchBoresightDegrees = 0.5;
    double headingBoresightDegrees = 0.6;
    Attitude boresightAngles(0, rollBoresightDegrees, pitchBoresightDegrees, headingBoresightDegrees);
    Eigen::Matrix3d boresight;
    Boresight::buildMatrix(boresight, boresightAngles);
    
    /*georef TRF*/
    GeoreferencingTRF geo;
    
    Eigen::Vector3d georefedPingTRF;
    geo.georeference(georefedPingTRF, attitude, position, ping, *svp, leverArm, boresight);
    
    /*georef LGF*/
    GeoreferencingLGF geoLGF;
    geoLGF.setCentroid(centroidPosition);
    
    Eigen::Vector3d georefedPingLGF;
    geoLGF.georeference(georefedPingLGF, attitude, position, ping, *svp, leverArm, boresight);
    
    /*Test georefTRF*/
    Eigen::Vector3d centroidTRF;
    CoordinateTransform::getPositionECEF(centroidTRF, centroidPosition);
    
    Eigen::Matrix3d ned2ecef;
    CoordinateTransform::ned2ecef(ned2ecef,centroidPosition);
    
    Eigen::Vector3d expectedGeorefedPingTRF = centroidTRF + ned2ecef*georefedPingLGF;
    
    double georefTestTreshold = 1e-5;
    REQUIRE(std::abs(expectedGeorefedPingTRF(0) - georefedPingTRF(0)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeorefedPingTRF(1) - georefedPingTRF(1)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeorefedPingTRF(2) - georefedPingTRF(2)) < georefTestTreshold);
    
    delete svp;
}

TEST_CASE("Georeference LGF with position and perpendicular unit vector ping and non-zero attitude and non-zero lever arm and non-zero boresight"){
    
    /*Build Centroid Position*/
    double centroidLatitude = 48.4525;
    double centroidLongitude = -68.5232;
    double centroidEllipsoidHeight = 15.401;
    Position centroidPosition(0, centroidLatitude, centroidLongitude, centroidEllipsoidHeight);
    
    /*Build attitude*/
    double roll = 1.0; // 0.017453293
    double pitch = 2.0; // 0.034906585
    double heading = 45.0; // 0.785398185
    Attitude attitude(0, roll, pitch, heading);

    /*Build Position*/
    double latitude = 48.4525;
    double longitude = -68.5232;
    double ellipsoidHeight = 15.401;
    Position position(0, latitude, longitude, ellipsoidHeight);

    /*Build Ping*/
    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;
    double surfaceSoundSpeed = 0;
    double twoWayTravelTime = 0.01;
    double alongTrackAngle = 0.0;
    double acrossTrackAngle = 0.0;
    
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

    /*Build SVP*/
    SoundVelocityProfile * svp = SoundVelocityProfileFactory::buildFreshWaterModel();
    ping.setSurfaceSoundSpeed(svp->getSpeeds()(0)); //important now that raytracing uses it

    /*Build lever arm*/
    Eigen::Vector3d leverArm(1, 2, 3);

    /*Build boresight matrix*/
    double rollBoresightDegrees = 0.4;
    double pitchBoresightDegrees = 0.5;
    double headingBoresightDegrees = 0.6;
    Attitude boresightAngles(0, rollBoresightDegrees, pitchBoresightDegrees, headingBoresightDegrees);
    Eigen::Matrix3d boresight;
    Boresight::buildMatrix(boresight, boresightAngles);
    
    /*georef LGF*/
    GeoreferencingLGF geo;
    geo.setCentroid(centroidPosition);
    
    Eigen::Vector3d georefedPing;
    geo.georeference(georefedPing, attitude, position, ping, *svp, leverArm, boresight);
    
    Eigen::Vector3d expectedGeorefedPing;
    expectedGeorefedPing << -0.2394900282, 2.2597419967, 10.3884422996;
    
    
    double georefTestTreshold = 1e-5;
    REQUIRE(std::abs(expectedGeorefedPing(0) - georefedPing(0)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeorefedPing(1) - georefedPing(1)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeorefedPing(2) - georefedPing(2)) < georefTestTreshold);
    
    delete svp;
}

TEST_CASE("Georeference TRF with position and perpendicular unit vector ping and non-zero attitude and non-zero lever-arm and non-zero boresight and centroid not colocated with position"){
    /*Build Centroid Position*/
    double centroidLatitude = 48.4525033333222;
    double centroidLongitude = -68.52320333332072;
    double centroidEllipsoidHeight = 15.267333333333;
    Position centroidPosition(0, centroidLatitude, centroidLongitude, centroidEllipsoidHeight);
    
    /*Build attitude*/
    double roll = 1.0; // 0.017453293
    double pitch = 2.0; // 0.034906585
    double heading = 45.0; // 0.785398185
    Attitude attitude(0, roll, pitch, heading);

    /*Build Position*/
    double latitude = 48.4525;
    double longitude = -68.5232;
    double ellipsoidHeight = 15.401;
    Position position(0, latitude, longitude, ellipsoidHeight);

    /*Build Ping*/
    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;
    double surfaceSoundSpeed = 0;
    double twoWayTravelTime = 0.01;
    double alongTrackAngle = 0.0;
    double acrossTrackAngle = 0.0;
    
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

    /*Build SVP*/
    SoundVelocityProfile * svp = SoundVelocityProfileFactory::buildFreshWaterModel();
    ping.setSurfaceSoundSpeed(svp->getSpeeds()(0)); //important now that raytracing uses it

    /*Build lever arm*/
    Eigen::Vector3d leverArm(1, 2, 3);

    /*Build boresight matrix*/
    double rollBoresightDegrees = 0.4;
    double pitchBoresightDegrees = 0.5;
    double headingBoresightDegrees = 0.6;
    Attitude boresightAngles(0, rollBoresightDegrees, pitchBoresightDegrees, headingBoresightDegrees);
    Eigen::Matrix3d boresight;
    Boresight::buildMatrix(boresight, boresightAngles);
    
    /*georef TRF*/
    GeoreferencingTRF geo;
    
    Eigen::Vector3d georefedPingTRF;
    geo.georeference(georefedPingTRF, attitude, position, ping, *svp, leverArm, boresight);
    
    /*georef LGF*/
    GeoreferencingLGF geoLGF;
    geoLGF.setCentroid(centroidPosition);
    
    Eigen::Vector3d georefedPingLGF;
    geoLGF.georeference(georefedPingLGF, attitude, position, ping, *svp, leverArm, boresight);
    
    /*Test georefTRF*/
    Eigen::Vector3d centroidTRF;
    CoordinateTransform::getPositionECEF(centroidTRF, centroidPosition);
    
    Eigen::Matrix3d ned2ecef;
    CoordinateTransform::ned2ecef(ned2ecef,centroidPosition);
    
    Eigen::Vector3d expectedGeorefedPingTRF = centroidTRF + ned2ecef*georefedPingLGF;
    
    double georefTestTreshold = 1e-5;
    REQUIRE(std::abs(expectedGeorefedPingTRF(0) - georefedPingTRF(0)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeorefedPingTRF(1) - georefedPingTRF(1)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeorefedPingTRF(2) - georefedPingTRF(2)) < georefTestTreshold);
    
    delete svp;
}

TEST_CASE("Georeference LGF with position and perpendicular unit vector ping and non-zero attitude and non-zero lever arm and non-zero boresight and centroid not colocated with position"){
    
    /*Build Centroid Position*/
    double centroidLatitude = 48.4525033333222;
    double centroidLongitude = -68.52320333332072;
    double centroidEllipsoidHeight = 15.267333333333;
    Position centroidPosition(0, centroidLatitude, centroidLongitude, centroidEllipsoidHeight);
    
    /*Build attitude*/
    double roll = 1.0; // 0.017453293
    double pitch = 2.0; // 0.034906585
    double heading = 45.0; // 0.785398185
    Attitude attitude(0, roll, pitch, heading);

    /*Build Position*/
    double latitude = 48.4525;
    double longitude = -68.5232;
    double ellipsoidHeight = 15.401;
    Position position(0, latitude, longitude, ellipsoidHeight);

    /*Build Ping*/
    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;
    double surfaceSoundSpeed = 0;
    double twoWayTravelTime = 0.01;
    double alongTrackAngle = 0.0;
    double acrossTrackAngle = 0.0;
    
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

    /*Build SVP*/
    SoundVelocityProfile * svp = SoundVelocityProfileFactory::buildFreshWaterModel();
    ping.setSurfaceSoundSpeed(svp->getSpeeds()(0)); //important now that raytracing uses it

    /*Build lever arm*/
    Eigen::Vector3d leverArm(1, 2, 3);

    /*Build boresight matrix*/
    double rollBoresightDegrees = 0.4;
    double pitchBoresightDegrees = 0.5;
    double headingBoresightDegrees = 0.6;
    Attitude boresightAngles(0, rollBoresightDegrees, pitchBoresightDegrees, headingBoresightDegrees);
    Eigen::Matrix3d boresight;
    Boresight::buildMatrix(boresight, boresightAngles);
    
    /*georef LGF*/
    GeoreferencingLGF geo;
    geo.setCentroid(centroidPosition);
    
    Eigen::Vector3d georefedPing;
    geo.georeference(georefedPing, attitude, position, ping, *svp, leverArm, boresight);
    
    Eigen::Vector3d expectedGeorefedPing;
    expectedGeorefedPing << -0.6101546444, 2.5063106918, 10.2547756501;
    
    double georefTestTreshold = 1e-5;
    REQUIRE(std::abs(expectedGeorefedPing(0) - georefedPing(0)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeorefedPing(1) - georefedPing(1)) < georefTestTreshold);
    REQUIRE(std::abs(expectedGeorefedPing(2) - georefedPing(2)) < georefTestTreshold);
    
    delete svp;
}

#endif /* GEOREFERENCINGTEST_HPP */

