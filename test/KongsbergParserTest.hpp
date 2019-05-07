/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   KongsbergParserTest.hpp
 * Author: emile
 *
 * Created on May 3, 2019, 2:02 PM
 */
#include "catch.hpp"
#include "../src/datagrams/DatagramEventHandler.hpp"
#include "../src/datagrams/kongsberg/KongsbergParser.hpp"

TEST_CASE("test the function KongsbergParser::getName")
{
    DatagramEventHandler handler;
    KongsbergParser parser(handler);
    REQUIRE(parser.getName(51)=="ExtraParameters 3");
    REQUIRE(parser.getName(107)=="Water column datagram");
    REQUIRE(parser.getName(1)=="Invalid tag");
    REQUIRE(parser.getName(67)=="Clock datagrams");
    REQUIRE(parser.getName(78)=="Raw range and beam angle 78 datagram");
    REQUIRE(parser.getName(56)=="Invalid tag");
    REQUIRE(parser.getName(89)=="Seabed image data 89 datagram");
    REQUIRE(parser.getName(102)=="Raw range and beam angle datagrams");
}