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

TEST_CASE("Georeference TRF with position only"){
  
    Eigen::Vector3d         georefedPing;
    
    Attitude                attitude(0,0,0,0);
    Position                position(0,48.4525,-68.5232,15.401);
    Ping                    ping(0,0,0,0,0,0.1,0.0,0.0);
    SoundVelocityProfile *  svp = SoundVelocityProfileFactory::buildFreshWaterModel();
    Eigen::Vector3d         leverArm(0,0,0);
    Eigen::Matrix3d         boresight = Eigen::Matrix3d::Identity();

    GeoreferencingTRF geo;
    geo.georeference(georefedPing,attitude,position,ping,*svp,leverArm,boresight);
    
    std::cout << "GEOREF TRF: " << georefedPing << std::endl;
    
    REQUIRE(false);
}

TEST_CASE("Georeference LGF with position only"){
  
    Eigen::Vector3d         georefedPing;
    
    Attitude                attitude(0,0,0,0);
    Position                position(0,48.4525,-68.5232,15.401);
    Ping                    ping(0,0,0,0,0,0.1,0,0);
    SoundVelocityProfile  *  svp = SoundVelocityProfileFactory::buildFreshWaterModel();
    Eigen::Vector3d         leverArm(0,0,0);
    Eigen::Matrix3d         boresight = Eigen::Matrix3d::Identity();

    GeoreferencingLGF geo;
    geo.georeference(georefedPing,attitude,position,ping,*svp,leverArm,boresight);
    
    std::cout << "GEOREF LGF: " << georefedPing << std::endl;
    
    REQUIRE(false);
}

TEST_CASE("Georeference TRF with position and perpendicular unit vector ping"){
    REQUIRE(false);
}

TEST_CASE("Georeference LGF with position and perpendicular unit vector ping"){
    REQUIRE(false);
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

