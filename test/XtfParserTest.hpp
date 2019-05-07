/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   XtfParserTest.hpp
 * Author: emile
 *
 * Created on May 3, 2019, 2:03 PM
 */
#include "catch.hpp"
#include "../src/datagrams/DatagramEventHandler.hpp"
#include "../src/datagrams/xtf/XtfParser.hpp"

TEST_CASE("test the function XtfParser::getName")
{
    DatagramEventHandler handler;
    XtfParser parser(handler);
    REQUIRE(parser.getName(3)=="XTF_HEADER_ATTITUDE");
    REQUIRE(parser.getName(56)=="Invalid tag");
    REQUIRE(parser.getName(108)=="XTF_HEADER_KLEINV4_DATA_PAGE");
    REQUIRE(parser.getName(78)=="XTF_HEADER_RESON_7018_WATERCOLUMN");
    REQUIRE(parser.getName(23)=="XTF_HEADER_GYRO");
    REQUIRE(parser.getName(28)=="XTF_HEADER_Q_MULTIBEAM");
    REQUIRE(parser.getName(69)=="XTF_HEADER_R2SONIC_FTS");
    REQUIRE(parser.getName(209)=="Invalid tag");
}