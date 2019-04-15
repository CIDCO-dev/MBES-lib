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
    svpW.setTimestamp(0);
    svpW.setLatitude(-1);
    svpW.setLongitude(-1);
    svpW.add(0,0);
    std::string file = "test.txt";
    svpW.write(file);
    SoundVelocityProfile svpR = SoundVelocityProfile();
    svpR.add(1,1);
    bool valide (svpR.read(file));
    REQUIRE(valide);
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
  svp.setTimestamp(0);
  std::string cont = svp.julianTime();
  REQUIRE(cont.compare("1970-1 0:0:0")==0);
}

/**Test the conversion of the latitude into text with the format direction dd:mm:ss*/
TEST_CASE("Test the latFormat method")
{
  SoundVelocityProfile svp = SoundVelocityProfile();
  svp.setLatitude(0);
  std::string cont = svp.latFormat(svp.getLatitude());
  REQUIRE(cont.compare("North 0:0:0")==0);
}

/**Test the conversion of the longitude into text with the format direction dd:mm:ss*/
TEST_CASE("Test the longFormat method")
{
  SoundVelocityProfile svp = SoundVelocityProfile();
  svp.setLongitude(0);
  std::string cont = svp.longFormat(svp.getLongitude());
  REQUIRE(cont.compare("East 0:0:0")==0);
}

/**Test the reading of the sound velocity profile timestamp, latitude and longitude*/
TEST_CASE("Read the sound velocity profile time, latitude, longitude")
{
    SoundVelocityProfile svp = SoundVelocityProfile();
    std::string text = "Section 1970-1 0:0:0 South 1:0:0 West 1:0:0";
    uint64_t time = 0;
    double lat = 0;
    double lon = 0;
    bool valide = svp.readTimeLatLong(text,time,lat,lon);
    REQUIRE(valide);
    REQUIRE(time==0);
    REQUIRE(lat==-1);
    REQUIRE(lon==-1);
}

/**Test if the SoundVelocityProfile class can get the depths and the speeds*/
TEST_CASE("Get speeds/depths"){
    SoundVelocityProfile svp = SoundVelocityProfile();
    svp.add(4,5);
    REQUIRE(svp.getDepths()(0) == 4);
    REQUIRE(svp.getSpeeds()(0) == 5);
}

