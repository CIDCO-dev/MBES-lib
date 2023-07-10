/* 
 * File:   XtfParserTest.hpp
 * Author: Patrick Charron-Morneau
 *
 */
#include "catch.hpp"
#include "../src/datagrams/DatagramEventHandler.hpp"
#include "../src/hydroblock/Hydroblock20Parser.hpp"

TEST_CASE("test invalid hydroblock2.0 directory")
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

TEST_CASE ("test the hydroblock2.0 parser with a valid datagram")
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
