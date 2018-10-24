#ifndef TEST_XCF_HPP
#define TEST_XCF_HPP

#include "catch.hpp"
#include "../src/datagrams/xtf/XtfTypes.hpp"

TEST_CASE( "Les structs XTF ont la bonne dimensions", "[xtfTypes]" ) {
    REQUIRE( sizeof(XtfFileHeader) == 1024 );
    REQUIRE( sizeof(XtfChanInfo) == 128 );
    REQUIRE( sizeof(XtfAttitudeData)+sizeof(XtfPacketHeader) == 64 );
    REQUIRE( sizeof(XtfNotesHeader) == 256-sizeof(XtfPacketHeader) );
    REQUIRE( sizeof(XtfRawSerialHeader) == 30 );
    REQUIRE( sizeof(XtfPingHeader) == 256-sizeof(XtfPacketHeader));
    REQUIRE( sizeof(XtfPingChanHeader) == 64);
    REQUIRE( sizeof(XtfHighSpeedSensor) == 64);
    REQUIRE( sizeof(XtfBeamXYZA) == 31);
    REQUIRE( sizeof(SNP0) == 74);
    REQUIRE( sizeof(SNP1) == 24);
    REQUIRE( sizeof(XtfPosRawNavigation)+sizeof(XtfPacketHeader) == 64);
    REQUIRE( sizeof(XtfQpsSingleBeam) == 54);
    REQUIRE( sizeof(XtfQpsMultiTxEntry) == 48);
    REQUIRE( sizeof(XtfQpsMbEntry) == 64 );
    REQUIRE( sizeof(XtfRawCustomHeader) == 64);
    REQUIRE( sizeof(XtfHeaderNavigation_type42) == 64 );
    REQUIRE( sizeof(XtfHeaderNavigation_type84) == 64 );
    REQUIRE( sizeof(XtfHeaderGyro) == 64);

}

#endif
