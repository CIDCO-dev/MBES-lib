/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   SoundVelocityProfileTest.cpp
 * Author: jordan
 *
 */

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <iomanip>
#include "catch.hpp"
#include "../src/SoundVelocityProfile.hpp"
#include "../src/utils/Exception.hpp"

/**Test the writing and the reading of a SVP file*/
TEST_CASE("Writing to SVP file"){
	//TODO: test prototype
    SoundVelocityProfile svpW = SoundVelocityProfile();
    svpW.add(0,0);
    std::string file = "test";
    svpW.write(file);
    SoundVelocityProfile svpR = SoundVelocityProfile();
    svpR.add(1,1);
    svpR.read(file);
    REQUIRE(svpW.getTimestamp() == svpR.getTimestamp());
    REQUIRE(svpW.getLatitude() == svpR.getLatitude());
    REQUIRE(svpW.getLongitude() == svpR.getLongitude());
    REQUIRE(svpW.getDepths() == svpR.getDepths());
    REQUIRE(svpW.getSpeeds() == svpR.getSpeeds());
}

/**Test the conversion of microEpoch into text with the format yyyy-ddd hh:mm:ss*/
TEST_CASE("Test the julianTime method")
{
  SoundVelocityProfile svp = SoundVelocityProfile();
  std::string cont = svp.julianTime();
  REQUIRE(cont.compare("1900-1 0:0:0")==0);
}

/**Test the conversion of the latitude into text with the format direction dd:mm:ss*/
TEST_CASE("Test the latFormat method")
{
  SoundVelocityProfile svp = SoundVelocityProfile();
  std::string cont = svp.latFormat(svp.getLatitude());
  REQUIRE(cont.compare("North 0:0:0")==0);
}

/**Test the conversion of the longitude into text with the format direction dd:mm:ss*/
TEST_CASE("Test the longFormat method")
{
  SoundVelocityProfile svp = SoundVelocityProfile();
  std::string cont = svp.longFormat(svp.getLongitude());
  REQUIRE(cont.compare("East 0:0:0")==0);
}

/**Test the reading of the sound velocity profile timestamp*/
TEST_CASE("Read the sound velocity profile time")
{
    SoundVelocityProfile svp = SoundVelocityProfile();
    std::string text = "1900-1 0:0:0 text";
    uint64_t time = svp.readTime(text);
    REQUIRE(time==0);
    REQUIRE(text.compare("text")==0);
}

/**Test the reading of the sound velocity profile latitude*/
TEST_CASE("Read the sound velocity profile latitude")
{
    SoundVelocityProfile svp = SoundVelocityProfile();
    std::string text = "South 1:0:0 text";
    double lat = svp.readLatLong(text);
    REQUIRE(lat==(-1));
    REQUIRE(text=="text");
}

/**Test the reading of the sound velocity profile longitude*/
TEST_CASE("Read the sound velocity profile longitude")
{
    SoundVelocityProfile svp = SoundVelocityProfile();
    std::string text = "West 1:0:0 text";
    double lon = svp.readLatLong(text);
    REQUIRE(lon==(-1));
    REQUIRE(text=="text");
}

/**Test if the SoundVelocityProfile class can get the depths and the speeds*/
TEST_CASE("Get speeds/depths"){
    SoundVelocityProfile svp = SoundVelocityProfile();
    svp.add(0,0);
    REQUIRE(svp.getDepths()(0) == 0);
    REQUIRE(svp.getSpeeds()(0) == 0);
}

