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
        excep = error->what();
    }
    REQUIRE(excep=="The two positions timestamp are the same");
    try
    {
        Interpolator::linearInterpolationByTime(0,0,0,1,0);
    }
    catch(Exception * error)
    {
        excep = error->what();
    }
    REQUIRE(excep=="The first position timestamp is higher than interpolation timestamp");
    try
    {
        Interpolator::linearInterpolationByTime(0,0,2,1,0);
    }
    catch(Exception * error)
    {
        excep = error->what();
    }
    REQUIRE(excep=="The first position timestamp is higher than the second position timestamp");
}

TEST_CASE("Test the linear interpolation")
{
    REQUIRE(abs(Interpolator::linearInterpolationByTime(10,10,20,10,30)-10)<1e-10);
    REQUIRE(abs(Interpolator::linearInterpolationByTime(45,60,90,80,90)-60)<1e-10);
    REQUIRE(abs(Interpolator::linearInterpolationByTime(120,345,634,408,678)-308.3333333333)<1e-10);
    REQUIRE(abs(Interpolator::linearInterpolationByTime(4567,17354,1500,1200,10000)-5002.92045454545)<1e-10);
    REQUIRE(abs(Interpolator::linearInterpolationByTime(0,0,1,0,2)-0)<1e-10);
    REQUIRE(abs(Interpolator::linearInterpolationByTime(4567,17354,10000,1200,1500)-379652.3333333333)<1e-10);
    REQUIRE(abs(Interpolator::linearInterpolationByTime(78,90,20,10,100)-79.3333333333)<1e-10);
    REQUIRE(abs(Interpolator::linearInterpolationByTime(7568,9087,60,10,100)-8411.88888888889)<1e-10);
}

TEST_CASE("Test the position interpolation")
{
    Position p1(10,10,10,10);
    Position p2(30,10,10,10);
    Position* pos = Interpolator::interpolatePosition(p1,p2,20);
    REQUIRE(abs(pos->getLatitude()-10)<1e-10);
    REQUIRE(abs(pos->getLongitude()-10)<1e-10);
    REQUIRE(abs(pos->getEllipsoidalHeight()-10)<1e-10);
    p1 = Position(10,10,45,0);
    p2 = Position(30,10,60,0);
    pos = Interpolator::interpolatePosition(p1,p2,20);
    REQUIRE(abs(pos->getLatitude()-10)<1e-10);
    REQUIRE(abs(pos->getLongitude()-52.5)<1e-10);
    REQUIRE(abs(pos->getEllipsoidalHeight()-0)<1e-10);
    p1 = Position(10,90,78,98);
    p2 = Position(100,180,150,123);
    pos = Interpolator::interpolatePosition(p1,p2,80);
    REQUIRE(abs(pos->getLatitude()-160)<1e-10);
    REQUIRE(abs(pos->getLongitude()-134)<1e-10);
    REQUIRE(abs(pos->getEllipsoidalHeight()-117.4444444444)<1e-10);
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
        excep = error->what();
    }
    REQUIRE(excep=="The two positions timestamp are the same");
    try
    {
        double test = Interpolator::linearAngleInterpolationByTime(0,0,0,1,0);
    }
    catch(Exception * error)
    {
        excep = error->what();
    }
    REQUIRE(excep=="The first position timestamp is higher than interpolation timestamp");
    try
    {
        double test = Interpolator::linearAngleInterpolationByTime(0,0,2,1,0);
    }
    catch(Exception * error)
    {
        excep = error->what();
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
        excep = error->what();
    }
    REQUIRE(excep == "The angles 0 and 180 have a difference of 180 degrees "
            "which means there are two possible answers at timestamp 50");
    excep = "";
    try
    {
        Interpolator::linearAngleInterpolationByTime(0,180,75,0,100); 
    }
    catch (Exception * error)
    {
        excep = error->what();
    }
    REQUIRE(excep == "The angles 0 and 180 have a difference of 180 degrees "
            "which means there are two possible answers at timestamp 75");
}

TEST_CASE("Test the angle interpolation")
{
    REQUIRE(std::abs(Interpolator::linearAngleInterpolationByTime(0,90,50,0,100)-45)<1e-10);
    REQUIRE(std::abs(Interpolator::linearAngleInterpolationByTime(0,270,50,0,100)-315)<1e-10);
    REQUIRE(std::abs(Interpolator::linearAngleInterpolationByTime(10,20,80,0,100)-18)<1e-10);
    REQUIRE(std::abs(Interpolator::linearAngleInterpolationByTime(0,350,50,0,100)-355)<1e-10);
    REQUIRE(std::abs(Interpolator::linearAngleInterpolationByTime(0,0,80,9,180)-0)<1e-10);
    REQUIRE(std::abs(Interpolator::linearAngleInterpolationByTime(70,90,50,0,100)-80)<1e-10);
    REQUIRE(std::abs(Interpolator::linearAngleInterpolationByTime(0,9,2,0,3)-6)<1e-10);
    REQUIRE(std::abs(Interpolator::linearAngleInterpolationByTime(0,70,50,0,100)-35)<1e-10);
}

TEST_CASE("Test the attitude interpolation")
{
    Attitude a1(0,0,0,0);
    Attitude a2(100,90,270,10);
    Attitude* att = Interpolator::interpolateAttitude(a1,a2,50);
    REQUIRE(abs(att->getRoll()-45)<1e-10);
    REQUIRE(abs(att->getPitch()-315)<1e-10);
    REQUIRE(abs(att->getHeading()-5)<1e-10);
    a1 = Attitude(0,0,0,0);
    a2 = Attitude(3,90,270,12);
    att = Interpolator::interpolateAttitude(a1,a2,2);
    REQUIRE(abs(att->getRoll()-60)<1e-10);
    REQUIRE(abs(att->getPitch()-300)<1e-10);
    REQUIRE(abs(att->getHeading()-8)<1e-10);
    a1 = Attitude(0,0,0,0);
    a2 = Attitude(100,90,270,10);
    att = Interpolator::interpolateAttitude(a1,a2,25);
    REQUIRE(abs(att->getRoll()-22.5)<1e-10);
    REQUIRE(abs(att->getPitch()-337.5)<1e-10);
    REQUIRE(abs(att->getHeading()-2.5)<1e-10);
}

