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
static void oldRayTrace(Eigen::Vector3d & raytracedPing,Ping & ping,SoundVelocityProfile & svp, Eigen::Matrix3d & boresightMatrix,Eigen::Matrix3d & imu2nav){

    /*
     * Compute launch vector
     */

    Eigen::Vector3d launchVectorSonar; //in sonar frame
    CoordinateTransform::sonar2cartesian(launchVectorSonar,ping.getAlongTrackAngle(),ping.getAcrossTrackAngle(), 1.0 ); 

    launchVectorSonar.normalize();

    //convert to navigation frame where the raytracing occurs
    Eigen::Vector3d launchVectorNav = imu2nav * (boresightMatrix * launchVectorSonar);

    double vNorm = sqrt(pow(launchVectorNav(0), 2)  + pow(launchVectorNav(1), 2));

    double sinAz= (vNorm >0)?launchVectorNav(0)/ vNorm : 0;
    double cosAz= (vNorm >0)?launchVectorNav(1)/ vNorm : 0;
    double beta0 = asin(launchVectorNav(2));

    //compute gradient for each layer
    std::vector<double> gradient;
    for (unsigned int k=0; k < svp.getSize()-1; k++){
        gradient.push_back( (svp.getSpeeds()[k+1]- svp.getSpeeds()[k])/(svp.getDepths()[k+1]- svp.getDepths()[k]) );
    }

    //Snell's law's coefficient, using the first layer
    double epsilon = cos(beta0)/svp.getSpeeds()[0];

   unsigned int N = 0;

   double sinBn     = 0;
   double sinBnm1   = 0;
   double cosBn     = 0;
   double cosBnm1   = 0;
   double DT        = 0;
   double dtt       = 0;
   double DZ        = 0;
   double DR        = 0;
   double radiusOfCurvature = 0;
   double xff       = 0;
   double zff       = 0;

    while((DT + dtt)<= (ping.getTwoWayTravelTime()/(double)2) && (N<svp.getSize()-1)){
            //update angles
            sinBnm1 = sqrt(1 - pow(epsilon*svp.getSpeeds()[N], 2));
            sinBn   = sqrt(1 - pow(epsilon*svp.getSpeeds()[N+1], 2));

            cosBnm1 = epsilon*svp.getSpeeds()[N];
            cosBn   = epsilon*svp.getSpeeds()[N+1];

            if (abs(gradient[N]) < 0.000001) //FIXME: use a global epsilon value?
            {
                    //celerity gradient is zero so constant celerity in this layer
                    //delta t, delta z and r for the layer N
                    DZ = svp.getDepths()[N+1] - svp.getDepths()[N];
                    dtt = DZ/(svp.getSpeeds()[N]*sinBn);
                    DR = cosBn*dtt*svp.getSpeeds()[N];
            }
            else {
                    // if not null gradient
                    //Radius of curvature
                    radiusOfCurvature = 1.0/(epsilon*gradient[N]);

                    //delta t, delta z and r for the layer N
                    dtt = abs( (1./abs(gradient[N]))*log( (svp.getSpeeds()[N+1]/svp.getSpeeds()[N])*( (1.0 + sinBnm1)/(1.0 + sinBn) ) ) );
                    DZ = radiusOfCurvature*(cosBn - cosBnm1);
                    DR = radiusOfCurvature*(sinBnm1 - sinBn);
            }

            //To ensure to work with the N-1 cumulated travel time
            if (DT + dtt <=  (ping.getTwoWayTravelTime()/(double)2))
            {
                    N = N+1;
                    xff = xff + DR;
                    zff = zff + DZ;
                    DT = DT + dtt;
            }

    }

    // Last Layer Propagation
    double dtf = (ping.getTwoWayTravelTime()/(double)2) - DT;
    double dxf = svp.getSpeeds()[N]*dtf*cosBn;
    double dzf = svp.getSpeeds()[N]*dtf*sinBn;

    // Output variable computation
    double Xf = xff + dxf;
    double Zf = zff + dzf;

    raytracedPing(0) = Xf*sinAz;
    raytracedPing(1) = Xf*cosAz;
    raytracedPing(2) = Zf;
}




TEST_CASE("describe this test") {
    
    /*Build an svp with a single constant speed layer*/    
    SoundVelocityProfile * svp = new SoundVelocityProfile();
    
    double depth1 = 0;
    double speed1 = 1500;
    
    double depth2 = 10;
    double speed2 = 1500;
    
    svp->add(depth1, speed1);
    svp->add(depth2, speed2);
    
    
    
    
    
    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;
    double surfaceSoundSpeed = 1500;
    double twoWayTravelTime = 2;
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
    
    
    
    Eigen::Vector3d raytracedPingOldFunction;
    oldRayTrace(raytracedPingOldFunction, ping, *svp, boresightMatrix, imu2nav);
    
    
    Eigen::Vector3d raytracedPingRefactored;
    Raytracing::rayTrace(raytracedPingRefactored, ping, *svp, boresightMatrix, imu2nav);
    
    std::cout << "raytracedPingOldFunction:" << std::endl;
    std::cout << raytracedPingOldFunction << std::endl;
    
    std::cout << "raytracedPingRefactored:" << std::endl;
    std::cout << raytracedPingRefactored << std::endl;
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

