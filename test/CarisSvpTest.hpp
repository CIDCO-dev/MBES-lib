/*
 * Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   CarisSvpTest.hpp
 * Author: jordan
 */

#ifndef CARISSVPTEST_HPP
#define CARISSVPTEST_HPP

#include "catch.hpp"
#include "../src/svp/CarisSvpFile.hpp"

TEST_CASE("CARIS SVP test") {
    /*Build first svp*/
    double lat1 = 48.0;
    double lon1 = -68.0;
    uint64_t t1 = 1566324919000000;
    double depth11 = 0.5; double speed11 = 1487.5;
    double depth12 = 1.5; double speed12 = 1487.4;
    
    SoundVelocityProfile * svp1 = new SoundVelocityProfile();
    svp1->setLatitude(lat1);
    svp1->setLongitude(lon1);
    svp1->setTimestamp(t1);
    svp1->add(depth11, speed11);
    svp1->add(depth12, speed12);
    
    /*Build second svp*/
    double lat2 = 58.0;
    double lon2 = -78.0;
    uint64_t t2 = 1566313837000000;
    double depth21 = 0.4; double speed21 = 1487.4;
    double depth22 = 1.4; double speed22 = 1487.2;
    
    SoundVelocityProfile * svp2 = new SoundVelocityProfile();
    svp2->setLatitude(lat2);
    svp2->setLongitude(lon2);
    svp2->setTimestamp(t2);
    svp2->add(depth21, speed21);
    svp2->add(depth22, speed22);
    
    std::vector<SoundVelocityProfile*> svps;
    svps.push_back(svp1);
    svps.push_back(svp2);
    
    /*test write and read*/
    CarisSvpFile svpsCarisOut;
    svpsCarisOut.setSvps(svps);
    
    std::string outputFilename = "build/test/output.svp";
    
    svpsCarisOut.writeSvpFile(outputFilename);
    
    CarisSvpFile svpsCarisIn;
    bool svpRead = svpsCarisIn.readSvpFile(outputFilename);
    
    REQUIRE(svpRead);
    
    double testThreshold = 1e-9;
    
    REQUIRE(svpsCarisIn.getSvps().size() == 2);
    
    SoundVelocityProfile * testSvp1 = svpsCarisIn.getSvps()[0];
    SoundVelocityProfile * testSvp2 = svpsCarisIn.getSvps()[1];
    
    REQUIRE(testSvp1->getTimestamp() == t1);
    REQUIRE(std::abs(testSvp1->getLatitude() - lat1) < testThreshold);
    REQUIRE(std::abs(testSvp1->getLongitude() - lon1) < testThreshold);
    REQUIRE(std::abs(testSvp1->getDepths()(0) - depth11) < testThreshold);
    REQUIRE(std::abs(testSvp1->getDepths()(1) - depth12) < testThreshold);
    REQUIRE(std::abs(testSvp1->getSpeeds()(0) - speed11) < testThreshold);
    REQUIRE(std::abs(testSvp1->getSpeeds()(1) - speed12) < testThreshold);
    
    REQUIRE(testSvp2->getTimestamp() == t2);
    REQUIRE(std::abs(testSvp2->getLatitude() - lat2) < testThreshold);
    REQUIRE(std::abs(testSvp2->getLongitude() - lon2) < testThreshold);
    REQUIRE(std::abs(testSvp2->getDepths()(0) - depth21) < testThreshold);
    REQUIRE(std::abs(testSvp2->getDepths()(1) - depth22) < testThreshold);
    REQUIRE(std::abs(testSvp2->getSpeeds()(0) - speed21) < testThreshold);
    REQUIRE(std::abs(testSvp2->getSpeeds()(1) - speed22) < testThreshold);
}

#endif /* CARISSVPTEST_HPP */

