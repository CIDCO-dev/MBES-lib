/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   S7kTypesTest.hpp
 * Author: emile
 *
 * Created on May 14, 2019, 10:05 AM
 */

#ifndef S7KTYPESTEST_HPP
#define S7KTYPESTEST_HPP
#include "catch.hpp"
#include "../src/datagrams/s7k/S7kTypes.hpp"

TEST_CASE( "Les structs S7k ont la bonne dimensions", "[xtfTypes]" ) {
    REQUIRE( sizeof(S7kTime) == 10 );
    REQUIRE( sizeof(S7kDataRecordFrame) == 64 );
    REQUIRE( sizeof(S7kFileHeader) == 316 );
    REQUIRE( sizeof(S7kFileHeaderRecordDatum) == 6 );
    REQUIRE( sizeof(S7kFileHeaderOptionalData) == 12 );
    REQUIRE( sizeof(S7kPosition) == 37 );
    REQUIRE( sizeof(S7kDepth) == 8 );
    REQUIRE( sizeof(S7kNavigation) == 41 );
    REQUIRE( sizeof(S7kAttitudeRTH) == 1 );
    REQUIRE( sizeof(S7kAttitudeRD) == 18 );
    REQUIRE( sizeof(S7kRawDetectionDataRTH) == 99 );
    REQUIRE( sizeof(S7kRawDetectionDataRD) == 26 );
    REQUIRE( sizeof(S7kSonarSettings) == 156 );
    REQUIRE( sizeof(S7kSoundVelocity) == 12 );
    REQUIRE( sizeof(S7kCtdRTH) == 36 );
    REQUIRE( sizeof(S7kCtdRD) == 20 );
}
#endif /* S7KTYPESTEST_HPP */

