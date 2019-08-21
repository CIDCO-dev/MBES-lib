/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   SvpStrategyTest.hpp
 * Author: jordan
 *
 * Created on August 19, 2019, 3:15 PM
 */

#ifndef SVPSTRATEGYTEST_HPP
#define SVPSTRATEGYTEST_HPP

#include "catch.hpp"
#include "../src/svp/SoundVelocityProfile.hpp"
#include "../src/svp/SvpSelectionStrategy.hpp"
#include "../src/svp/SvpNearestByTime.hpp"
#include "../src/svp/SvpNearestByLocation.hpp"
#include "../src/utils/Constants.hpp"

TEST_CASE("SVP selection test") {
    
    /*Build first svp*/
    double lat1 = 48.0;
    double lon1 = -68.0;
    uint64_t t1 = 1000;
    
    SoundVelocityProfile * svp1 = new SoundVelocityProfile();
    svp1->setLatitude(lat1);
    svp1->setLongitude(lon1);
    svp1->setTimestamp(t1);
    
    /*Build second svp*/
    double lat2 = 58.0;
    double lon2 = -78.0;
    uint64_t t2 = 2000;
    
    SoundVelocityProfile * svp2 = new SoundVelocityProfile();
    svp2->setLatitude(lat2);
    svp2->setLongitude(lon2);
    svp2->setTimestamp(t2);
    
    /*Build strategies*/
    SvpSelectionStrategy * locationStrat = new SvpNearestByLocation();
    locationStrat->addSvp(svp1);
    locationStrat->addSvp(svp2);
    
    SvpSelectionStrategy * timeStrat = new SvpNearestByTime();
    timeStrat->addSvp(svp1);
    timeStrat->addSvp(svp2);
    
    /*Build first ping*/
    Position p1(0, 58.1, -77.9, 0);
    
    uint64_t microEpoch1 = 1350;
    long id1 = 0;
    uint32_t quality1 = 0;
    double intensity1 = 0;
    double surfaceSoundSpeed1 = 1446.4250488;
    double twoWayTravelTime1 = 0.0091418369 * 2;
    double alongTrackAngle1 = 0.0;
    double acrossTrackAngle1 = 0.7031931281 * R2D;

    Ping ping1(
            microEpoch1,
            id1,
            quality1,
            intensity1,
            surfaceSoundSpeed1,
            twoWayTravelTime1,
            alongTrackAngle1,
            acrossTrackAngle1
            );
    
    /*Build second ping*/
    Position p2(0, 48.1, -67.9, 0);
    
    uint64_t microEpoch2 = 1750;
    long id2 = 0;
    uint32_t quality2 = 0;
    double intensity2 = 0;
    double surfaceSoundSpeed2 = 1446.4250488;
    double twoWayTravelTime2 = 0.0091418369 * 2;
    double alongTrackAngle2 = 0.0;
    double acrossTrackAngle2 = 0.7031931281 * R2D;

    Ping ping2(
            microEpoch2,
            id2,
            quality2,
            intensity2,
            surfaceSoundSpeed2,
            twoWayTravelTime2,
            alongTrackAngle2,
            acrossTrackAngle2
            );
    
    /*Select svps*/
    SoundVelocityProfile * testLocationSvp1 = locationStrat->chooseSvp(p1, ping1);
    SoundVelocityProfile * testTimeSvp1 = timeStrat->chooseSvp(p1, ping1);
    SoundVelocityProfile * testLocationSvp2 = locationStrat->chooseSvp(p2, ping2);
    SoundVelocityProfile * testTimeSvp2 = timeStrat->chooseSvp(p2, ping2);
    
    double testThreshold = 1e-9;
    
    //ping1 is closest to svp2
    REQUIRE(std::abs(testLocationSvp1->getLatitude() - lat2) < testThreshold);
    REQUIRE(std::abs(testLocationSvp1->getLongitude() - lon2) < testThreshold);
    
    //ping2 is closest to svp1
    REQUIRE(std::abs(testLocationSvp2->getLatitude() - lat1) < testThreshold);
    REQUIRE(std::abs(testLocationSvp2->getLongitude() - lon1) < testThreshold);
    
    
    //ping1 is neared in time to svp1
    REQUIRE(std::abs(testTimeSvp1->getLatitude() - lat1) < testThreshold);
    REQUIRE(std::abs(testTimeSvp1->getLongitude() - lon1) < testThreshold);
    
    //ping2 is neared in time to svp2
    REQUIRE(std::abs(testTimeSvp2->getLatitude() - lat2) < testThreshold);
    REQUIRE(std::abs(testTimeSvp2->getLongitude() - lon2) < testThreshold);
            
    
    
    
    
}


#endif /* SVPSTRATEGYTEST_HPP */

