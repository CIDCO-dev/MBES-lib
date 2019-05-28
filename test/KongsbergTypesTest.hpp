/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   KongsbergTypesTest.hpp
 * Author: emile
 *
 * Created on May 14, 2019, 9:16 AM
 */

#ifndef KONGSBERGTYPESTEST_HPP
#define KONGSBERGTYPESTEST_HPP
#include "catch.hpp"
#include "../src/datagrams/kongsberg/KongsbergTypes.hpp"

TEST_CASE( "Les structs Kongsberg ont la bonne dimensions", "[xtfTypes]" ) {
    REQUIRE( sizeof(KongsbergHeader) == 20 );
    REQUIRE( sizeof(KongsbergAttitudeEntry) == 12 );
    REQUIRE( sizeof(KongsbergPositionDatagram) == 18 );
    REQUIRE( sizeof(KongsbergSoundSpeedProfile) == 12 );
    REQUIRE( sizeof(KongsbergSoundSpeedProfileEntry) == 8 );
    REQUIRE( sizeof(KongsbergRangeAndBeam78) == 16 );
    REQUIRE( sizeof(KongsbergRangeAndBeam78TxEntry) == 24 );
    REQUIRE( sizeof(KongsbergRangeAndBeam78RxEntry) == 16 );
}

#endif /* KONGSBERGTYPESTEST_HPP */

