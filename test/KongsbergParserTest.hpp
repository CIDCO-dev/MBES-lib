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
    REQUIRE(parser.getName(48)=="PU Id output datagrams");
    REQUIRE(parser.getName(49)=="PU Status output");
    REQUIRE(parser.getName(51)=="ExtraParameters 3");
    REQUIRE(parser.getName(53)=="Seabed image datagram");
    REQUIRE(parser.getName(65)=="Attitude datagram");
    REQUIRE(parser.getName(66)=="PU BIST result output");
    REQUIRE(parser.getName(67)=="Clock datagrams");
    REQUIRE(parser.getName(68)=="Depth datagram");
    REQUIRE(parser.getName(69)=="Single beam echo sounder depth datagram");
    REQUIRE(parser.getName(70)=="Raw range and beam angle datagrams");
    REQUIRE(parser.getName(71)=="Surface sound speed datagram");
    REQUIRE(parser.getName(72)=="Heading datagrams");
    REQUIRE(parser.getName(73)=="Installation parameters");
    REQUIRE(parser.getName(74)=="Mechanical transducer tilt datagrams");
    REQUIRE(parser.getName(75)=="Central beams echogram");
    REQUIRE(parser.getName(78)=="Raw range and beam angle 78 datagram");
    REQUIRE(parser.getName(79)=="Quality factor datagram 79");
    REQUIRE(parser.getName(80)=="Position datagrams");
    REQUIRE(parser.getName(82)=="Runtime parameters");
    REQUIRE(parser.getName(84)=="Tide datagram");
    REQUIRE(parser.getName(85)=="Sound speed profile datagram");
    REQUIRE(parser.getName(87)=="Kongsberg Maritime SSP output datagram");
    REQUIRE(parser.getName(88)=="XYZ 88");
    REQUIRE(parser.getName(89)=="Seabed image data 89 datagram");
    REQUIRE(parser.getName(102)=="Raw range and beam angle datagrams");
    REQUIRE(parser.getName(104)=="Depth (pressure) or height datagram");
    REQUIRE(parser.getName(105)=="Installation parameters");
    REQUIRE(parser.getName(107)=="Water column datagram");
    REQUIRE(parser.getName(108)=="Extra detections");
    REQUIRE(parser.getName(110)=="Network attitude velocity datagram 110");
    REQUIRE(parser.getName(114)=="Installation parameters or remote information");
    REQUIRE(parser.getName(115)=="Invalid tag");
}

TEST_CASE ("test the Kongsberg parser with a file who doesn't exist")
{
    DatagramEventHandler handler;
    KongsbergParser parser(handler);
    std::string file("blabla.all");
    std::string excep = "";
    try
    {
        parser.parse(file);
        REQUIRE(false);
    }
    catch(Exception * error)
    {
        excep = error->what();
        REQUIRE(true);
    }
}

TEST_CASE ("test the Kongsberg parser with a invalid datagram")
{
    DatagramEventHandler handler;
    KongsbergParser parser(handler);
    std::string file("test.txt");
    std::string excep = "";
    try
    {
        parser.parse(file);
        REQUIRE(false);
    }
    catch(Exception * error)
    {
        excep = error->what();
        REQUIRE(true);
    }
}

TEST_CASE ("test the Kongsberg parser with a valid datagram")
{
    DatagramEventHandler handler;
    KongsbergParser parser(handler);
    std::string file("test/data/all/0008_20160909_135801_Panopee.all");
    std::string excep = "";
    try
    {
        parser.parse(file);
        REQUIRE(true);
    }
    catch(Exception * error)
    {
        excep = error->what();
        REQUIRE(false);
    }
}
