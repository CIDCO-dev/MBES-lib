/*
 * Copyright 2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   BoundingBoxTest.hpp
 * Author: Jordan McManus
 */

#ifndef BOUNDINGBOXTEST_HPP
#define BOUNDINGBOXTEST_HPP

#include "catch.hpp"
#include "../src/datagrams/DatagramParserFactory.hpp"
#include "../src/examples/BoundingBoxPrinter.hpp"

TEST_CASE("bounding-box .s7k test") {
    BoundingBoxPrinter  printer;
    
    std::string fileName ="test/data/s7k/20141016_150519_FJ-Saucier.s7k";
    
    DatagramParser * parser = DatagramParserFactory::build(fileName,printer);
    parser->parse(fileName);
    
    REQUIRE(printer.getMinimumLatitude() > 50.6);
    REQUIRE(printer.getMaximumLatitude() < 50.7);
    REQUIRE(printer.getMinimumLongitude() > -63.3);
    REQUIRE(printer.getMaximumLongitude() < -63.2);
}

TEST_CASE("bounding-box .all test") {
    BoundingBoxPrinter  printer;
    
    std::string fileName ="test/data/all/0008_20160909_135801_Panopee.all";
    
    DatagramParser * parser = DatagramParserFactory::build(fileName,printer);
    parser->parse(fileName);
    
    REQUIRE(printer.getMinimumLatitude() > 48.3);
    REQUIRE(printer.getMaximumLatitude() < 48.4);
    
    REQUIRE(printer.getMinimumLongitude() > -4.5);
    REQUIRE(printer.getMaximumLongitude() < -4.4);
}

TEST_CASE("bounding-box .xtf test") {
    BoundingBoxPrinter  printer;
    
    std::string fileName ="test/data/xtf/0008_20160909_EM2040C_MIBAC - 0001.xtf";
    
    DatagramParser * parser = DatagramParserFactory::build(fileName,printer);
    parser->parse(fileName);
    
    REQUIRE(printer.getMinimumLatitude() > 48.3);
    REQUIRE(printer.getMaximumLatitude() < 48.4);
    
    REQUIRE(printer.getMinimumLongitude() > -4.5);
    REQUIRE(printer.getMaximumLongitude() < -4.4);
}

TEST_CASE("bounding-box starfish .xtf test") {
    BoundingBoxPrinter  printer;
    
    std::string fileName ="test/data/xtfStarFish/22_07_2020_C1.xtf";
    
    DatagramParser * parser = DatagramParserFactory::build(fileName,printer);
    parser->parse(fileName);
    
    REQUIRE(printer.getMinimumLatitude() > 46.8);
    REQUIRE(printer.getMaximumLatitude() < 46.9);
    
    REQUIRE(printer.getMinimumLongitude() > -71.2);
    REQUIRE(printer.getMaximumLongitude() < -71.1);
}

TEST_CASE("bounding-box sidescan .xtf test") {
    BoundingBoxPrinter  printer;
    
    std::string fileName ="test/data/xtf/Line-001-0856.sidescan.xtf";
    
    DatagramParser * parser = DatagramParserFactory::build(fileName,printer);
    parser->parse(fileName);
    
    REQUIRE(printer.getMinimumLatitude() > 48.5);
    REQUIRE(printer.getMaximumLatitude() < 48.6);
    
    REQUIRE(printer.getMinimumLongitude() > -68.5);
    REQUIRE(printer.getMaximumLongitude() < -68.4);
}

#endif /* BOUNDINGBOXTEST_HPP */

