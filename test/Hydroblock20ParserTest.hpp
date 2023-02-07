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
#include "../src/hydroblock/Hydroblock20Parser.hpp"

TEST_CASE("test invalid directory")
{
    DatagramEventHandler handler;
    Hydroblock20Parser parser(handler);
    std::string file("./");
    std::string excep = "";
    try
    {
        parser.parse(file, true);
        REQUIRE(false);
    }
    catch(Exception * error)
    {
        excep = error->what();
        REQUIRE(true);
    }
}

TEST_CASE ("test the XTF parser with a valid datagram")
{
    DatagramEventHandler handler;
    Hydroblock20Parser parser(handler);
    std::string file("test/data/hydroblock2.0/");
    std::string excep = "";
    try
    {
        parser.parse(file, true);
        REQUIRE(true);
    }
    catch(Exception * error)
    {
        excep = error->what();
        REQUIRE(false);
    }
}
