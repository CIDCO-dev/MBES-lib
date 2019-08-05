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
    REQUIRE(parser.getName(0)=="XTF_HEADER_SONAR");
    REQUIRE(parser.getName(1)=="XTF_HEADER_NOTES");
    REQUIRE(parser.getName(2)=="XTF_HEADER_BATHY");
    REQUIRE(parser.getName(3)=="XTF_HEADER_ATTITUDE");
    REQUIRE(parser.getName(4)=="XTF_HEADER_FORWARD");
    REQUIRE(parser.getName(5)=="XTF_HEADER_ELAC");
    REQUIRE(parser.getName(6)=="XTF_HEADER_RAW_SERIAL");
    REQUIRE(parser.getName(7)=="XTF_HEADER_EMBED_HEAD");
    REQUIRE(parser.getName(8)=="XTF_HEADER_HIDDEN_SONAR");
    REQUIRE(parser.getName(9)=="XTF_HEADER_SEAVIEW_PROCESSED_BATHY");
    REQUIRE(parser.getName(10)=="XTF_HEADER_SEAVIEW_DEPTHS");
    REQUIRE(parser.getName(11)=="XTF_HEADER_RSVD_HIGHSPEED_SENSOR");
    REQUIRE(parser.getName(12)=="XTF_HEADER_ECHOSTRENGTH");
    REQUIRE(parser.getName(13)=="XTF_HEADER_GEOREC");
    REQUIRE(parser.getName(14)=="XTF_HEADER_KLEIN_RAW_BATHY");
    REQUIRE(parser.getName(15)=="XTF_HEADER_HIGHSPEED_SENSOR2");
    REQUIRE(parser.getName(16)=="XTF_HEADER_ELAC_XSE");
    REQUIRE(parser.getName(17)=="XTF_HEADER_BATHY_XYZA");
    REQUIRE(parser.getName(18)=="XTF_HEADER_K5000_BATHY_IQ");
    REQUIRE(parser.getName(19)=="XTF_HEADER_BATHY_SNIPPET");
    REQUIRE(parser.getName(20)=="XTF_HEADER_GPS");
    REQUIRE(parser.getName(21)=="XTF_HEADER_STAT");
    REQUIRE(parser.getName(22)=="XTF_HEADER_SINGLEBEAM");
    REQUIRE(parser.getName(23)=="XTF_HEADER_GYRO");
    REQUIRE(parser.getName(24)=="XTF_HEADER_TRACKPOINT");
    REQUIRE(parser.getName(25)=="XTF_HEADER_MULTIBEAM");
    REQUIRE(parser.getName(26)=="XTF_HEADER_Q_SINGLEBEAM");
    REQUIRE(parser.getName(27)=="XTF_HEADER_Q_MULTITX");
    REQUIRE(parser.getName(28)=="XTF_HEADER_Q_MULTIBEAM");
    REQUIRE(parser.getName(50)=="XTF_HEADER_TIME");
    REQUIRE(parser.getName(60)=="XTF_HEADER_BENTHOS_CAATI_SARA");
    REQUIRE(parser.getName(61)=="XTF_HEADER_7125");
    REQUIRE(parser.getName(62)=="XTF_HEADER_7125_SNIPPET");
    REQUIRE(parser.getName(65)=="XTF_HEADER_QINSY_R2SONIC_BATHY");
    REQUIRE(parser.getName(66)=="XTF_HEADER_QINSY_R2SONIC_FTS");
    REQUIRE(parser.getName(68)=="XTF_HEADER_R2SONIC_BATHY");
    REQUIRE(parser.getName(69)=="XTF_HEADER_R2SONIC_FTS");
    REQUIRE(parser.getName(70)=="XTF_HEADER_CODA_ECHOSCOPE_DATA");
    REQUIRE(parser.getName(71)=="XTF_HEADER_CODA_ECHOSCOPE_CONFIG");
    REQUIRE(parser.getName(72)=="XTF_HEADER_CODA_ECHOSCOPE_IMAGE");
    REQUIRE(parser.getName(73)=="XTF_HEADER_EDGETECH_4600");
    REQUIRE(parser.getName(78)=="XTF_HEADER_RESON_7018_WATERCOLUMN");
    REQUIRE(parser.getName(100)=="XTF_HEADER_POSITION");
    REQUIRE(parser.getName(102)=="XTF_HEADER_BATHY_PROC");
    REQUIRE(parser.getName(103)=="XTF_HEADER_ATTITUDE_PROC");
    REQUIRE(parser.getName(104)=="XTF_HEADER_SINGLEBEAM_PROC");
    REQUIRE(parser.getName(105)=="XTF_HEADER_AUX_PROC");
    REQUIRE(parser.getName(106)=="XTF_HEADER_KLEIN3000_DATA_PAGE");
    REQUIRE(parser.getName(107)=="XTF_HEADER_POS_RAW_NAVIGATION");
    REQUIRE(parser.getName(108)=="XTF_HEADER_KLEINV4_DATA_PAGE");
    REQUIRE(parser.getName(200)=="XTF_HEADER_USERDEFINED");
    REQUIRE(parser.getName(300)=="Invalid tag");
}

TEST_CASE ("test the XTF parser with a file who doesn't exist")
{
    DatagramEventHandler handler;
    XtfParser parser(handler);
    std::string file("blabla.xtf");
    std::string excep = "";
    try
    {
        parser.parse(file);
    }
    catch(Exception * error)
    {
        excep = error->getMessage();
    }
    REQUIRE(excep=="File not found");
}

TEST_CASE ("test the XTF parser with a invalid file")
{
    DatagramEventHandler handler;
    XtfParser parser(handler);
    std::string file("test.txt");
    std::string excep = "";
    try
    {
        parser.parse(file);
    }
    catch(Exception * error)
    {
        excep = error->getMessage();
    }
    REQUIRE(excep=="Couldn't read from file");
}

TEST_CASE ("test the XTF parser with a invalid datagram")
{
    DatagramEventHandler handler;
    XtfParser parser(handler);
    std::string file("test/data/xtf/0008_20160909_EM2040C_MIBAC - 0001.xtf");
    std::string excep = "";
    try
    {
        parser.parse(file);
    }
    catch(Exception * error)
    {
        excep = error->getMessage();
    }
    REQUIRE(excep=="");
}