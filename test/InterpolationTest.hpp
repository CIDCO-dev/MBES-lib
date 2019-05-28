/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   InterpolationTest.hpp
 * Author: emile
 *
 * Created on May 27, 2019, 10:00 AM
 */

#include "catch.hpp"
#include "../src/math/Interpolation.hpp"

TEST_CASE("Test the linear interpolation with invalid timestamp")
{
    std::string excep;
    try
    {
        Interpolator::linearInterpolationByTime(0,0,0,0,0);
    }
    catch(Exception * error)
    {
        excep = error->getMessage();
    }
    REQUIRE(excep=="The two positions timestamp are the same");
    try
    {
        Interpolator::linearInterpolationByTime(0,0,0,1,0);
    }
    catch(Exception * error)
    {
        excep = error->getMessage();
    }
    REQUIRE(excep=="The first position timestamp is higher than interpolation timestamp");
    try
    {
        Interpolator::linearInterpolationByTime(0,0,2,1,0);
    }
    catch(Exception * error)
    {
        excep = error->getMessage();
    }
    REQUIRE(excep=="The first position timestamp is higher than the second position timestamp");
}

TEST_CASE("Test the linear interpolation")
{
    REQUIRE(abs(10-Interpolator::linearInterpolationByTime(10,10,20,10,30))<1e-10);
    REQUIRE(abs(60-Interpolator::linearInterpolationByTime(45,60,90,80,90))<1e-10);
    REQUIRE(abs(308.3333333333-Interpolator::linearInterpolationByTime(120,345,634,408,678))<1e-10);
    REQUIRE(abs(5002.92045454545-Interpolator::linearInterpolationByTime(4567,17354,1500,1200,10000))<1e-10);
    REQUIRE(abs(0-Interpolator::linearInterpolationByTime(0,0,1,0,2))<1e-10);
    REQUIRE(abs(379652.3333333333-Interpolator::linearInterpolationByTime(4567,17354,10000,1200,1500))<1e-10);
    REQUIRE(abs(79.3333333333-Interpolator::linearInterpolationByTime(78,90,20,10,100))<1e-10);
    REQUIRE(abs(8411.88888888889-Interpolator::linearInterpolationByTime(7568,9087,60,10,100))<1e-10);
}

TEST_CASE("Test the position interpolation")
{
    Position p1(10,10,10,10);
    Position p2(30,10,10,10);
    Position* pos = Interpolator::interpolatePosition(p1,p2,20);
    REQUIRE(abs(10-pos->getLatitude())<1e-10);
    REQUIRE(abs(10-pos->getLongitude())<1e-10);
    REQUIRE(abs(10-pos->getEllipsoidalHeight())<1e-10);
    p1 = Position(10,10,45,0);
    p2 = Position(30,10,60,0);
    pos = Interpolator::interpolatePosition(p1,p2,20);
    REQUIRE(abs(10-pos->getLatitude())<1e-10);
    REQUIRE(abs(52.5-pos->getLongitude())<1e-10);
    REQUIRE(abs(0-pos->getEllipsoidalHeight())<1e-10);
    p1 = Position(10,90,78,98);
    p2 = Position(100,180,150,123);
    pos = Interpolator::interpolatePosition(p1,p2,80);
    REQUIRE(abs(160-pos->getLatitude())<1e-10);
    REQUIRE(abs(134-pos->getLongitude())<1e-10);
    REQUIRE(abs(117.4444444444-pos->getEllipsoidalHeight())<1e-10);
}

TEST_CASE("Test the angle interpolation with invalid angle")
{
    std::string excep;
    try
    {
        Interpolator::linearAngleInterpolationByTime(-1,180,50,0,100);
    }
    catch (Exception * error)
    {
        excep = error->getMessage();
    }
    REQUIRE(excep=="Angles need to be between 0 (inclusive) and 360 (exclusive) degrees");
    excep="";
    try
    {
        Interpolator::linearAngleInterpolationByTime(360,180,50,0,100);
    }
    catch (Exception * error)
    {
        excep = error->getMessage();
    }
    REQUIRE(excep=="Angles need to be between 0 (inclusive) and 360 (exclusive) degrees");
    excep="";
    try
    {
        Interpolator::linearAngleInterpolationByTime(0,-1,50,0,100);
    }
    catch (Exception * error)
    {
        excep = error->getMessage();
    }
    REQUIRE(excep=="Angles need to be between 0 (inclusive) and 360 (exclusive) degrees");
    excep="";
    try
    {
        Interpolator::linearAngleInterpolationByTime(0,360,50,0,100);
    }
    catch (Exception * error)
    {
        excep = error->getMessage();
    }
    REQUIRE(excep=="Angles need to be between 0 (inclusive) and 360 (exclusive) degrees");
}

TEST_CASE("Test the angle interpolation with invalid timestamp")
{
    std::string excep = "";
    try
    {
        double test = Interpolator::linearAngleInterpolationByTime(0,0,0,0,0);
    }
    catch(Exception * error)
    {
        excep = error->getMessage();
    }
    REQUIRE(excep=="The two positions timestamp are the same");
    try
    {
        double test = Interpolator::linearAngleInterpolationByTime(0,0,0,1,0);
    }
    catch(Exception * error)
    {
        excep = error->getMessage();
    }
    REQUIRE(excep=="The first position timestamp is higher than interpolation timestamp");
    try
    {
        double test = Interpolator::linearAngleInterpolationByTime(0,0,2,1,0);
    }
    catch(Exception * error)
    {
        excep = error->getMessage();
    }
    REQUIRE(excep=="The first position timestamp is higher than the second position timestamp");
}

TEST_CASE("Test the angle interpolation with a difference of 180 degrees")
{
    std::string excep = "";
    try
    {
        Interpolator::linearAngleInterpolationByTime(0,180,50,0,100); 
    }
    catch (Exception * error)
    {
        excep = error->getMessage();
    }
    REQUIRE(excep == "The angles 0 and 180 have a difference of 180 degrees "
            "witch mean there is two possible answer at the timestamp 50: 90 and 270\n");
    excep = "";
    try
    {
        Interpolator::linearAngleInterpolationByTime(0,180,75,0,100); 
    }
    catch (Exception * error)
    {
        excep = error->getMessage();
    }
    REQUIRE(excep == "The angles 0 and 180 have a difference of 180 degrees "
            "witch mean there is two possible answer at the timestamp 75: 135 and 225\n");
}

TEST_CASE("Test the angle interpolation")
{
    REQUIRE(std::abs(45-Interpolator::linearAngleInterpolationByTime(0,90,50,0,100))<1e-10);
    REQUIRE(std::abs(315-Interpolator::linearAngleInterpolationByTime(0,270,50,0,100))<1e-10);
    REQUIRE(std::abs(18-Interpolator::linearAngleInterpolationByTime(10,20,80,0,100))<1e-10);
    REQUIRE(std::abs(355-Interpolator::linearAngleInterpolationByTime(0,350,50,0,100))<1e-10);
    REQUIRE(std::abs(0-Interpolator::linearAngleInterpolationByTime(0,0,80,9,180))<1e-10);
    REQUIRE(std::abs(80-Interpolator::linearAngleInterpolationByTime(70,90,50,0,100))<1e-10);
    REQUIRE(std::abs(6-Interpolator::linearAngleInterpolationByTime(0,9,2,0,3))<1e-10);
    REQUIRE(std::abs(35-Interpolator::linearAngleInterpolationByTime(0,70,50,0,100))<1e-10);
}

TEST_CASE("Test the angle radians interpolation")
{
    REQUIRE(std::abs(45*M_PI/180-Interpolator::linearAngleRadiansInterpolationByTime(0*M_PI/180,90*M_PI/180,50,0,100))<1e-10);
    REQUIRE(std::abs(315*M_PI/180-Interpolator::linearAngleRadiansInterpolationByTime(0*M_PI/180,270*M_PI/180,50,0,100))<1e-10);
    REQUIRE(std::abs(18*M_PI/180-Interpolator::linearAngleRadiansInterpolationByTime(10*M_PI/180,20*M_PI/180,80,0,100))<1e-10);
    REQUIRE(std::abs(355*M_PI/180-Interpolator::linearAngleRadiansInterpolationByTime(0*M_PI/180,350*M_PI/180,50,0,100))<1e-10);
    REQUIRE(std::abs(0*M_PI/180-Interpolator::linearAngleRadiansInterpolationByTime(0*M_PI/180,0*M_PI/180,80,9,180))<1e-10);
    REQUIRE(std::abs(80*M_PI/180-Interpolator::linearAngleRadiansInterpolationByTime(70*M_PI/180,90*M_PI/180,50,0,100))<1e-10);
    REQUIRE(std::abs(6*M_PI/180-Interpolator::linearAngleRadiansInterpolationByTime(0*M_PI/180,9*M_PI/180,2,0,3))<1e-10);
    REQUIRE(std::abs(35*M_PI/180-Interpolator::linearAngleRadiansInterpolationByTime(0*M_PI/180,70*M_PI/180,50,0,100))<1e-10);
}

TEST_CASE("Test the attitude interpolation")
{
    Attitude a1(0,0,0,0);
    Attitude a2(100,90,270,10);
    Attitude* att = Interpolator::interpolateAttitude(a1,a2,50);
    REQUIRE(abs(45-att->getRoll())<1e-10);
    REQUIRE(abs(315-att->getPitch())<1e-10);
    REQUIRE(abs(5-att->getHeading())<1e-10);
    a1 = Attitude(0,0,0,0);
    a2 = Attitude(3,90,270,12);
    att = Interpolator::interpolateAttitude(a1,a2,2);
    REQUIRE(abs(60-att->getRoll())<1e-10);
    REQUIRE(abs(300-att->getPitch())<1e-10);
    REQUIRE(abs(8-att->getHeading())<1e-10);
    a1 = Attitude(0,0,0,0);
    a2 = Attitude(100,90,270,10);
    att = Interpolator::interpolateAttitude(a1,a2,25);
    REQUIRE(abs(22.5-att->getRoll())<1e-10);
    REQUIRE(abs(337.5-att->getPitch())<1e-10);
    REQUIRE(abs(2.5-att->getHeading())<1e-10);
}

