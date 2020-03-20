/*
 * Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   SidescanPingTest.hpp
 * Author: Jordan McManus <jordan.mcmanus@cidco.ca>
 *
 * Created on March 20, 2020, 3:30 PM
 */

#ifndef SIDESCANPINGTEST_HPP
#define SIDESCANPINGTEST_HPP

#include "catch.hpp"
#include <vector>
#include "../src/sidescan/SidescanPing.hpp"
#include "../src/Attitude.hpp"
#include "../src/Position.hpp"

TEST_CASE("SidescanPing test") {
    
    uint64_t testTimestamp = 42;
    
    
    double testLatitude = 45.5;
    double testLongitude = -68.4;
    double testHeight = 30.2;
    Position* position = new Position(testTimestamp, testLatitude, testLongitude, testHeight);
    
    
    double testRoll = -4.3;
    double testPitch = 2.4;
    double testHeading = 35.7;
    Attitude* attitude = new Attitude(testTimestamp, testRoll, testPitch, testHeading);
    
    int testChannelNumber = 1;
    
    
    
    double testSample1 = 34.5;
    double testSample2 = 24.5;
    
    std::vector<double> testSampleVector;
    testSampleVector.push_back(testSample1);
    testSampleVector.push_back(testSample2);
    
    double testDistancePerSample = 24.3;
    
    SidescanPing ping;
    
    ping.setAttitude(attitude);
    ping.setPosition(position);
    ping.setTimestamp(testTimestamp);
    ping.setSamples(testSampleVector);
    ping.setDistancePerSample(testDistancePerSample);
    ping.setChannelNumber(testChannelNumber);
    
    double epsPing = 1e-9;
    
    REQUIRE(std::abs(ping.getAttitude()->getRoll() - testRoll) < epsPing);
    REQUIRE(std::abs(ping.getAttitude()->getPitch() - testPitch) < epsPing);
    REQUIRE(std::abs(ping.getAttitude()->getHeading() - testHeading) < epsPing);
    
    REQUIRE(std::abs(ping.getPosition()->getLatitude() - testLatitude) < epsPing);
    REQUIRE(std::abs(ping.getPosition()->getLongitude() - testLongitude) < epsPing);
    REQUIRE(std::abs(ping.getPosition()->getEllipsoidalHeight() - testHeight) < epsPing);
    
    REQUIRE(ping.getChannelNumber() == testChannelNumber);
    
    REQUIRE(std::abs(ping.getDistancePerSample() - testDistancePerSample) < epsPing);
    
    REQUIRE(std::abs(ping.getSamples()[0] - testSample1) < epsPing);
    REQUIRE(std::abs(ping.getSamples()[1] - testSample2) < epsPing);
    
    REQUIRE(ping.getTimestamp() == testTimestamp);
    
    //delete position;
    //delete attitude;
}


#endif /* SIDESCANPINGTEST_HPP */

