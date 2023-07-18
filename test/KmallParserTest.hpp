/* 
 * File:   KmallParserTest.hpp
 * Author: Patrick CM
 *
 * Created on July 10th 2023
 */
#include "catch.hpp"
#include "../src/datagrams/DatagramEventHandler.hpp"
#include "../src/datagrams/kmall/KmallParser.hpp"

TEST_CASE("test the Kmall parser with a invalid datagram") {
    DatagramEventHandler handler;
    KmallParser parser(handler);
    std::string file("test.txt");
    std::string excep = "";
    try {
        parser.parse(file);
        REQUIRE(false);
    } catch (Exception * error) {
        excep = error->what();
        REQUIRE(true);
    }
}

TEST_CASE("test the Kmall parser with a valid datagram") {
    DatagramEventHandler handler;
    KmallParser parser(handler);
    std::string file("test/data/kmall/0002_20230524_175652.kmall");
    std::string excep = "";
    try {
        parser.parse(file);
        REQUIRE(true);
    } catch (Exception * error) {
        excep = error->what();
        REQUIRE(false);
    }
}
