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
    
    REQUIRE(printer.getMinimumLatitude() <= 90);
    REQUIRE(printer.getMinimumLatitude() >= -90);
    
    REQUIRE(printer.getMaximumLatitude() <= 90);
    REQUIRE(printer.getMaximumLatitude() >= -90);
    
    REQUIRE(printer.getMinimumLongitude() <= 180);
    REQUIRE(printer.getMinimumLongitude() >= -180);
    
    REQUIRE(printer.getMaximumLongitude() <= 180);
    REQUIRE(printer.getMaximumLongitude() >= -180);
}

TEST_CASE("bounding-box .all test") {
    BoundingBoxPrinter  printer;
    
    std::string fileName ="test/data/all/0008_20160909_135801_Panopee.all";
    
    DatagramParser * parser = DatagramParserFactory::build(fileName,printer);
    parser->parse(fileName);
    
    REQUIRE(printer.getMinimumLatitude() <= 90);
    REQUIRE(printer.getMinimumLatitude() >= -90);
    
    REQUIRE(printer.getMaximumLatitude() <= 90);
    REQUIRE(printer.getMaximumLatitude() >= -90);
    
    REQUIRE(printer.getMinimumLongitude() <= 180);
    REQUIRE(printer.getMinimumLongitude() >= -180);
    
    REQUIRE(printer.getMaximumLongitude() <= 180);
    REQUIRE(printer.getMaximumLongitude() >= -180);
}

TEST_CASE("bounding-box .xtf test") {
    BoundingBoxPrinter  printer;
    
    std::string fileName ="test/data/xtf/0008_20160909_EM2040C_MIBAC - 0001.xtf";
    
    DatagramParser * parser = DatagramParserFactory::build(fileName,printer);
    parser->parse(fileName);
    
    REQUIRE(printer.getMinimumLatitude() <= 90);
    REQUIRE(printer.getMinimumLatitude() >= -90);
    
    REQUIRE(printer.getMaximumLatitude() <= 90);
    REQUIRE(printer.getMaximumLatitude() >= -90);
    
    REQUIRE(printer.getMinimumLongitude() <= 180);
    REQUIRE(printer.getMinimumLongitude() >= -180);
    
    REQUIRE(printer.getMaximumLongitude() <= 180);
    REQUIRE(printer.getMaximumLongitude() >= -180);
}

TEST_CASE("bounding-box starfish .xtf test") {
    BoundingBoxPrinter  printer;
    
    std::string fileName ="test/data/xtfStarFish/22_07_2020_C1.xtf";
    
    DatagramParser * parser = DatagramParserFactory::build(fileName,printer);
    parser->parse(fileName);
    
    REQUIRE(printer.getMinimumLatitude() <= 90);
    REQUIRE(printer.getMinimumLatitude() >= -90);
    
    REQUIRE(printer.getMaximumLatitude() <= 90);
    REQUIRE(printer.getMaximumLatitude() >= -90);
    
    REQUIRE(printer.getMinimumLongitude() <= 180);
    REQUIRE(printer.getMinimumLongitude() >= -180);
    
    REQUIRE(printer.getMaximumLongitude() <= 180);
    REQUIRE(printer.getMaximumLongitude() >= -180);
}

#endif /* BOUNDINGBOXTEST_HPP */

