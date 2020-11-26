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

/*
 * This is the raytrace function before refactoring. It is used to test the 
 * refactored function
 */
static void oldRayTrace(Eigen::Vector3d & raytracedPing, Ping & ping, SoundVelocityProfile & svp, Eigen::Matrix3d & boresightMatrix, Eigen::Matrix3d & imu2nav) {

    /*
     * Compute launch vector
     */

    Eigen::Vector3d launchVectorSonar; //in sonar frame
    CoordinateTransform::sonar2cartesian(launchVectorSonar, ping.getAlongTrackAngle(), ping.getAcrossTrackAngle(), 1.0);

    launchVectorSonar.normalize();

    //convert to navigation frame where the raytracing occurs
    Eigen::Vector3d launchVectorNav = imu2nav * (boresightMatrix * launchVectorSonar);

    double vNorm = sqrt(pow(launchVectorNav(0), 2) + pow(launchVectorNav(1), 2));

    double sinAz = (vNorm > 0) ? launchVectorNav(0) / vNorm : 0;
    double cosAz = (vNorm > 0) ? launchVectorNav(1) / vNorm : 0;
    double beta0 = asin(launchVectorNav(2));

    //compute gradient for each layer
    std::vector<double> gradient;
    for (unsigned int k = 0; k < svp.getSize() - 1; k++) {
        gradient.push_back((svp.getSpeeds()[k + 1] - svp.getSpeeds()[k]) / (svp.getDepths()[k + 1] - svp.getDepths()[k]));
    }

    //Snell's law's coefficient, using the first layer
    double epsilon = cos(beta0) / svp.getSpeeds()[0];

    unsigned int N = 0;

    double sinBn = 0;
    double sinBnm1 = 0;
    double cosBn = 0;
    double cosBnm1 = 0;
    double DT = 0;
    double dtt = 0;
    double DZ = 0;
    double DR = 0;
    double radiusOfCurvature = 0;
    double xff = 0;
    double zff = 0;

    while ((DT + dtt) <= (ping.getTwoWayTravelTime() / (double) 2) && (N < svp.getSize() - 1)) {
        //update angles
        sinBnm1 = sqrt(1 - pow(epsilon * svp.getSpeeds()[N], 2));
        sinBn = sqrt(1 - pow(epsilon * svp.getSpeeds()[N + 1], 2));

        cosBnm1 = epsilon * svp.getSpeeds()[N];
        cosBn = epsilon * svp.getSpeeds()[N + 1];

        if (abs(gradient[N]) < 0.000001) //FIXME: use a global epsilon value?
        {
            //celerity gradient is zero so constant celerity in this layer
            //delta t, delta z and r for the layer N
            DZ = svp.getDepths()[N + 1] - svp.getDepths()[N];
            dtt = DZ / (svp.getSpeeds()[N] * sinBn);
            DR = cosBn * dtt * svp.getSpeeds()[N];
        } else {
            // if not null gradient
            //Radius of curvature
            radiusOfCurvature = 1.0 / (epsilon * gradient[N]);

            //delta t, delta z and r for the layer N
            dtt = abs((1. / abs(gradient[N])) * log((svp.getSpeeds()[N + 1] / svp.getSpeeds()[N])*((1.0 + sinBnm1) / (1.0 + sinBn))));
            DZ = radiusOfCurvature * (cosBn - cosBnm1);
            DR = radiusOfCurvature * (sinBnm1 - sinBn);
        }

        //To ensure to work with the N-1 cumulated travel time
        if (DT + dtt <= (ping.getTwoWayTravelTime() / (double) 2)) {
            N = N + 1;
            xff = xff + DR;
            zff = zff + DZ;
            DT = DT + dtt;
        }
    }

    // Last Layer Propagation
    double dtf = (ping.getTwoWayTravelTime() / (double) 2) - DT;
    double dxf = svp.getSpeeds()[N] * dtf*cosBn;
    double dzf = svp.getSpeeds()[N] * dtf*sinBn;

    // Output variable computation
    double Xf = xff + dxf;
    double Zf = zff + dzf;

    raytracedPing(0) = Xf*sinAz;
    raytracedPing(1) = Xf*cosAz;
    raytracedPing(2) = Zf;
}

TEST_CASE("ray tracing with transducer depth case") {

    std::string svpFilePath1 = "test/data/rayTracingTestData/SVP-0.svp";
    CarisSvpFile svps1;
    svps1.readSvpFile(svpFilePath1);
    SoundVelocityProfile * svp1 = svps1.getSvps()[0];
    
    std::string svpFilePath2 = "test/data/rayTracingTestData/SVP-0_1.svp";
    CarisSvpFile svps2;
    svps2.readSvpFile(svpFilePath2);
    SoundVelocityProfile * svp2 = svps2.getSvps()[0];
    
    double depth = svp2->getDepths()(0);
    double speed = svp2->getSpeeds()(0);
    
    Eigen::Matrix3d boresightMatrix = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d imu2nav = Eigen::Matrix3d::Identity();

    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;
    double surfaceSoundSpeed = speed;
    double oneWayTravelTime = 0.004;
    double twoWayTravelTime = 2*oneWayTravelTime;
    double alongTrackAngle = 0.0;
    double acrossTrackAngle = 45.0; //45 degrees

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

    ping.setTransducerDepth(depth); // set ping to svp depth
    
    Eigen::Vector3d rayOldRayTraced;
    oldRayTrace(rayOldRayTraced, ping, *svp2, boresightMatrix, imu2nav);

    Eigen::Vector3d ray;
    Raytracing::rayTrace(ray, ping, *svp1, boresightMatrix, imu2nav);
    
    double rayTestTreshold = 1e-9;
    REQUIRE(std::abs(rayOldRayTraced(0) - ray(0)) < rayTestTreshold);
    REQUIRE(std::abs(rayOldRayTraced(1) - ray(1)) < rayTestTreshold);
    REQUIRE(std::abs(rayOldRayTraced(2) - ray(2)) < rayTestTreshold);
}

TEST_CASE("ray tracing without transducer depth test case") {

    std::string svpFilePath = "test/data/rayTracingTestData/SVP-0.svp";
    CarisSvpFile svps;
    svps.readSvpFile(svpFilePath);
    SoundVelocityProfile * svp = svps.getSvps()[0];

    double depth1 = svp->getDepths()(0);
    double speed1 = svp->getSpeeds()(0);
    
    Eigen::Matrix3d boresightMatrix = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d imu2nav = Eigen::Matrix3d::Identity();

    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;
    double surfaceSoundSpeed = speed1;
    double oneWayTravelTime = 0.004;
    double twoWayTravelTime = 2*oneWayTravelTime;
    double alongTrackAngle = 0.0;
    double acrossTrackAngle = 45.0; //45 degrees

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

    // By setting the transducer depth to svp first sample depth
    // we make sure that the refactored ray tracing algorithm
    // gives the same result as the old raytracing
    ping.setTransducerDepth(depth1); // set ping to svp depth
    
    Eigen::Vector3d rayOldRayTraced;
    oldRayTrace(rayOldRayTraced, ping, *svp, boresightMatrix, imu2nav);

    Eigen::Vector3d ray;
    Raytracing::rayTrace(ray, ping, *svp, boresightMatrix, imu2nav);
    
    double rayTestTreshold = 1e-9;
    REQUIRE(std::abs(rayOldRayTraced(0) - ray(0)) < rayTestTreshold);
    REQUIRE(std::abs(rayOldRayTraced(1) - ray(1)) < rayTestTreshold);
    REQUIRE(std::abs(rayOldRayTraced(2) - ray(2)) < rayTestTreshold);
}

TEST_CASE("Launch vector straight down, one way travel time is 1 second, constant c=1500") {

    /*Build an svp with a single constant speed layer*/
    SoundVelocityProfile * svp = new SoundVelocityProfile();

    double depth1 = 0;
    double speed1 = 1500;

    double depth2 = 10; // svp ends before bottom
    double speed2 = 1500;

    svp->add(depth1, speed1);
    svp->add(depth2, speed2);

    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;
    double surfaceSoundSpeed = 1500;
    double twoWayTravelTime = 2; // oneWayTravelTime is 1 second
    // launch angle is straight down
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

    Eigen::Matrix3d boresightMatrix = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d imu2nav = Eigen::Matrix3d::Identity();

    Eigen::Vector3d raytracedPingRefactored;
    Raytracing::rayTrace(raytracedPingRefactored, ping, *svp, boresightMatrix, imu2nav);

    double rayTestTreshold = 1e-9;
    REQUIRE(std::abs(0 - raytracedPingRefactored(0)) < rayTestTreshold);
    REQUIRE(std::abs(0 - raytracedPingRefactored(1)) < rayTestTreshold);
    REQUIRE(std::abs(1500 - raytracedPingRefactored(2)) < rayTestTreshold);
}

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
    double rollBoresightDegrees = 0.62;
    double pitchBoresightDegrees = 0.0;
    double headingBoresightDegrees = 0.0;
    Attitude boresightAngles(0, rollBoresightDegrees, pitchBoresightDegrees, headingBoresightDegrees);
    Eigen::Matrix3d boresightMatrix;
    Boresight::buildMatrix(boresightMatrix, boresightAngles);

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

    double rayTestTreshold = 2e-2; // 2 cm difference
    REQUIRE(std::abs(expectedRay(0) - ray(0)) < rayTestTreshold);
    REQUIRE(std::abs(expectedRay(1) - ray(1)) < rayTestTreshold);
    REQUIRE(std::abs(expectedRay(2) - ray(2)) < rayTestTreshold);
}

TEST_CASE("Ray tracing with transducer depth test") {
    
    /*Build an svp with a 3 layers*/
    SoundVelocityProfile * svp = new SoundVelocityProfile();

    double depth1 = 5;
    double speed1 = 1500;

    double depth2 = 7;
    double speed2 = 1550;
    
    double transducerDepth1 = 4;
    double transducerDepth2 = 6;
    double transducerDepth3 = 8;

    svp->add(depth1, speed1);
    svp->add(depth2, speed2);
    
    /*Build 3 Pings*/
    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;
    double surfaceSoundSpeed1 = 1475;
    double surfaceSoundSpeed2 = 1525;
    double surfaceSoundSpeed3 = 1575;
    double twoWayTravelTime = 0.02 * 2;
    double alongTrackAngle = 0.0;
    double acrossTrackAngle = 45.0;

    Ping ping1(
            microEpoch,
            id,
            quality,
            intensity,
            surfaceSoundSpeed1,
            twoWayTravelTime,
            alongTrackAngle,
            acrossTrackAngle
            );
    
    Ping ping2(
            microEpoch,
            id,
            quality,
            intensity,
            surfaceSoundSpeed2,
            twoWayTravelTime,
            alongTrackAngle,
            acrossTrackAngle
            );
    
    Ping ping3(
            microEpoch,
            id,
            quality,
            intensity,
            surfaceSoundSpeed3,
            twoWayTravelTime,
            alongTrackAngle,
            acrossTrackAngle
            );
    
    
    ping1.setTransducerDepth(transducerDepth1);  // shallower than first SVP sample
    ping2.setTransducerDepth(transducerDepth2);  // between the 2 SVP samples
    ping3.setTransducerDepth(transducerDepth3); // deeper than last SVP sample
    
    
    Eigen::Matrix3d boresightMatrix = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d imu2nav = Eigen::Matrix3d::Identity();
    
    /* Perform the ray tracing*/
    Eigen::Vector3d ray1;
    Raytracing::rayTrace(ray1, ping1, *svp, boresightMatrix, imu2nav);
    
    Eigen::Vector3d ray2;
    Raytracing::rayTrace(ray2, ping2, *svp, boresightMatrix, imu2nav);
    
    Eigen::Vector3d ray3;
    Raytracing::rayTrace(ray3, ping2, *svp, boresightMatrix, imu2nav);
    
    std::cout << "ray1" << std::endl;
    std::cout << ray1 << std::endl;
    
    std::cout << "ray2" << std::endl;
    std::cout << ray2 << std::endl;
    
    std::cout << "ray3" << std::endl;
    std::cout << ray3 << std::endl;
    
    
}



#endif /* RAYTRACINGTEST_HPP */

