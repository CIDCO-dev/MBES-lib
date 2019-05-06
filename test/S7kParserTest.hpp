/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   S7kParserTest.hpp
 * Author: emile
 *
 * Created on May 3, 2019, 2:03 PM
 */
#include "catch.hpp"
#include "../src/datagrams/DatagramEventHandler.hpp"
#include "../src/datagrams/s7k/S7kParser.hpp"

TEST_CASE("test the function getName")
{
    DatagramEventHandler handler;
    S7kParser parser(handler);
    REQUIRE(parser.getName(1004)=="Custom Attitude Information");
    REQUIRE(parser.getName(1020)=="Sonar Installation Identifiers");
    REQUIRE(parser.getName(2000)=="Invalid tag");
    REQUIRE(parser.getName(7000)=="7k Sonar Settings");
    REQUIRE(parser.getName(7007)=="7k Side Scan Data");
    REQUIRE(parser.getName(7030)=="Sonar Installation Parameters");
    REQUIRE(parser.getName(7050)=="7k System Events");
    REQUIRE(parser.getName(7700)=="Invalid tag");
}
