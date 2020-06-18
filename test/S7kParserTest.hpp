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

TEST_CASE("test the function S7kParser::getName") {
    DatagramEventHandler handler;
    S7kParser parser(handler);
    REQUIRE(parser.getName(1000) == "Reference Point");
    REQUIRE(parser.getName(1001) == "Sensor Offset Position");
    REQUIRE(parser.getName(1002) == "Sensor Offset Position Calibrated");
    REQUIRE(parser.getName(1003) == "Position");
    REQUIRE(parser.getName(1004) == "Custom Attitude Information");
    REQUIRE(parser.getName(1005) == "Tide");
    REQUIRE(parser.getName(1006) == "Altitude");
    REQUIRE(parser.getName(1007) == "Motion Over Ground");
    REQUIRE(parser.getName(1008) == "Depth");
    REQUIRE(parser.getName(1009) == "Sound Velocity Profile");
    REQUIRE(parser.getName(1010) == "CTD");
    REQUIRE(parser.getName(1011) == "Geodesy");
    REQUIRE(parser.getName(1012) == "Roll Pitch Heave");
    REQUIRE(parser.getName(1013) == "Heading");
    REQUIRE(parser.getName(1014) == "Survey Line");
    REQUIRE(parser.getName(1015) == "Navigation");
    REQUIRE(parser.getName(1016) == "Attitude");
    REQUIRE(parser.getName(1017) == "Pan Tilt");
    REQUIRE(parser.getName(1020) == "Sonar Installation Identifiers");
    REQUIRE(parser.getName(2004) == "Sonar Pipe Environment");
    REQUIRE(parser.getName(7000) == "7k Sonar Settings");
    REQUIRE(parser.getName(7001) == "7k Configuration");
    REQUIRE(parser.getName(7002) == "7k Match Filter");
    REQUIRE(parser.getName(7003) == "7k Firmware and Hardware Configuration");
    REQUIRE(parser.getName(7004) == "7k Beam Geometry");
    REQUIRE(parser.getName(7006) == "7k Bathymetric Data");
    REQUIRE(parser.getName(7007) == "7k Side Scan Data");
    REQUIRE(parser.getName(7008) == "7k Generic Water Column Data");
    REQUIRE(parser.getName(7010) == "TVG Values");
    REQUIRE(parser.getName(7011) == "7k Image Data");
    REQUIRE(parser.getName(7012) == "7k Ping Motion Data");
    REQUIRE(parser.getName(7017) == "7k Detection Data Setup");
    REQUIRE(parser.getName(7018) == "7k Beamformed Data");
    REQUIRE(parser.getName(7019) == "Vernier Processing Data");
    REQUIRE(parser.getName(7021) == "7k Built-In Test Environment Data");
    REQUIRE(parser.getName(7022) == "7kCenter Version");
    REQUIRE(parser.getName(7023) == "8k Wet End Version");
    REQUIRE(parser.getName(7027) == "7k RAW Detection Data");
    REQUIRE(parser.getName(7028) == "7k Snippet Data");
    REQUIRE(parser.getName(7030) == "Sonar Installation Parameters");
    REQUIRE(parser.getName(7031) == "7k Built-In Test Environment Data (Summary)");
    REQUIRE(parser.getName(7041) == "Compressed Beamformed Magnitude Data");
    REQUIRE(parser.getName(7042) == "Compressed Watercolumn Data");
    REQUIRE(parser.getName(7048) == "7k Calibrated Beam Data");
    REQUIRE(parser.getName(7050) == "7k System Events");
    REQUIRE(parser.getName(7051) == "7k System Event Message");
    REQUIRE(parser.getName(7052) == "RDR Recording Status");
    REQUIRE(parser.getName(7053) == "7k Subscriptions");
    REQUIRE(parser.getName(7055) == "Calibration Status");
    REQUIRE(parser.getName(7057) == "Calibrated Side-Scan Data");
    REQUIRE(parser.getName(7058) == "Calibrated Snippet Data");
    REQUIRE(parser.getName(7059) == "MB2 specific status");
    REQUIRE(parser.getName(7200) == "7k File Header");
    REQUIRE(parser.getName(7300) == "7k File Catalog Record");
    REQUIRE(parser.getName(7400) == "7k Time Message");
    REQUIRE(parser.getName(7500) == "7k Remote Control");
    REQUIRE(parser.getName(7501) == "7k Remote Control Acknowledge");
    REQUIRE(parser.getName(7502) == "7k Remote Control Not Acknowledge");
    REQUIRE(parser.getName(7503) == "Remote Control Sonar Settings");
    REQUIRE(parser.getName(7504) == "7P Common System Settings");
    REQUIRE(parser.getName(7510) == "SV Filtering");
    REQUIRE(parser.getName(7511) == "System Lock Status");
    REQUIRE(parser.getName(7610) == "7k Sound Velocity");
    REQUIRE(parser.getName(7611) == "7k Absorption Loss");
    REQUIRE(parser.getName(7612) == "7k Spreading Loss");
    REQUIRE(parser.getName(7700) == "Invalid tag");
}

TEST_CASE("test the S7k parser with a file who doesn't exist") {
    DatagramEventHandler handler;
    S7kParser parser(handler);
    std::string file("blabla.s7k");
    std::string excep = "";
    try {
        parser.parse(file);
        REQUIRE(false);
    }    catch (Exception * error) {
        excep = error->what();
        REQUIRE(true);
    }
}

TEST_CASE("test the S7k parser with a invalid file") {
    DatagramEventHandler handler;
    S7kParser parser(handler);
    std::string file("test.txt");
    std::string excep = "";
    try {
        parser.parse(file);
        REQUIRE(false);
    }    catch (Exception * error) {
        excep = error->what();
        REQUIRE(true);
    }
}

TEST_CASE("test the S7k parser with a valid file") {
    DatagramEventHandler handler;
    S7kParser parser(handler);
    std::string file("test/data/s7k/20141016_150519_FJ-Saucier.s7k");
    std::string excep = "";
    try {
        parser.parse(file);
        REQUIRE(true);
    }    catch (Exception * error) {
        excep = error->what();
        REQUIRE(false);
    }
}

TEST_CASE("test S7k file that has 1012 and 1013 packets for attitude") {

    //file with bad checksums
    std::string file("test/data/s7k/20170529_111841_Seabat.s7k");

    class TestHandler : public DatagramEventHandler {
        public:

            void processAttitude(uint64_t microEpoch, double heading, double pitch, double roll) {
                attitudes.push_back(Attitude(microEpoch, roll, pitch, heading));
            };
            
            unsigned int getNumberOfAttitudes() {
                return attitudes.size();
            }
        protected:
            /**vector of attitudes*/
            std::vector<Attitude> attitudes;
    };

    TestHandler handler;
    S7kParser parser(handler);
    
    parser.parse(file);
    
    REQUIRE(handler.getNumberOfAttitudes() == 1102);
}
