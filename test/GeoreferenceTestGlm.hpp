/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/* 
 * File:   georeferenceTest.hpp
 * Author: glm
 *
 * Created on April 25, 2019, 9:13 AM
 */


#include "catch.hpp"
#include "../src/utils/Exception.hpp"
#include "../src/georeferencing/Georeferencing.hpp"
#include "../src/svp/SoundVelocityProfileFactory.hpp"
#include "../src/Ping.hpp"

#define POSITION_PRECISION 0.00000001

TEST_CASE("Georeference TRF with position and downward ping only"){
    Eigen::Vector3d         georefedPing;
    
    Attitude                attitude(0,0,0,0);
    Position                position(0,48.4525,-68.5232,15.401);
    Ping                    ping(0,0,0,0,0,0.01,0.0,0.0);
    SoundVelocityProfile *  svp = SoundVelocityProfileFactory::buildFreshWaterModel();
    Eigen::Vector3d         leverArm(0,0,0);
    Eigen::Matrix3d         boresight = Eigen::Matrix3d::Identity();

    GeoreferencingTRF geo;
    geo.georeference(georefedPing,attitude,position,ping,*svp,leverArm,boresight);
    
    //std::cerr << "GEOREF TRF: " << georefedPing << std::endl;
    
    Position georefPosition(0,0,0,0);
    
    CoordinateTransform::convertECEFToLongitudeLatitudeElevation(georefedPing,georefPosition);
    
    std::cerr << "Final position: " << std::endl << georefPosition << std::endl << std::endl;

    REQUIRE(abs(georefPosition.getLongitude() - position.getLongitude()) < POSITION_PRECISION);
    REQUIRE(abs(georefPosition.getLatitude() - position.getLatitude()) < POSITION_PRECISION);
    REQUIRE(abs(georefPosition.getEllipsoidalHeight() - (position.getEllipsoidalHeight()-7.4)) < POSITION_PRECISION);
}

TEST_CASE("Georeference LGF with position and downward ping only"){
  
    Eigen::Vector3d         georefedPing;
    
    Attitude                attitude(0,0,0,0);
    Position                position(0,48.4525,-68.5232,15.401);
    Ping                    ping(0,0,0,0,0,0.01,0,0);
    SoundVelocityProfile  *  svp = SoundVelocityProfileFactory::buildFreshWaterModel();
    Eigen::Vector3d         leverArm(0,0,0);
    Eigen::Matrix3d         boresight = Eigen::Matrix3d::Identity();

    GeoreferencingLGF geo;
    geo.georeference(georefedPing,attitude,position,ping,*svp,leverArm,boresight);
    
    //std::cerr << "GEOREF LGF: " << std::endl << georefedPing << std::endl << std::endl;

    Eigen::Vector3d expectedPosition(0,0,7.4);
    REQUIRE(georefedPing.isApprox(expectedPosition,POSITION_PRECISION));
}

TEST_CASE("Georeference TRF with position and perpendicular unit vector ping and non-zero attitude"){
    
    REQUIRE(false);
}

TEST_CASE("Georeference LGF with position and perpendicular unit vector ping and non-zero attitude"){
    REQUIRE(false);
}

TEST_CASE("Georeference TRF with position and perpendicular unit vector ping and non-zero attitude and non-zero lever-arm"){
    REQUIRE(false);
}

TEST_CASE("Georeference LGF with position and perpendicular unit vector ping and non-zero attitude and non-zero lever arm"){
    REQUIRE(false);
}

TEST_CASE("Georeference TRF with position and perpendicular unit vector ping and non-zero attitude and non-zero lever-arm and non-zero boresight"){
    REQUIRE(false);
}

TEST_CASE("Georeference LGF with position and perpendicular unit vector ping and non-zero attitude and non-zero lever arm and non-zero boresight"){
    REQUIRE(false);
}

