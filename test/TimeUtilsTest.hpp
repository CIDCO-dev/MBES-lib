/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/* 
 * File:   TimeUtilsTest.hpp
 * Author: emile
 *
 * Created on May 13, 2019, 9:27 AM
 */
#include "../src/utils/TimeUtils.hpp"
#include "catch.hpp"

/*

TEST_CASE("test the build time with the parameters year, month, day, hour, minute, seconds, milliseconds and microseconds")
{    
    uint64_t timestamp = TimeUtils::build_time(2019,0,1,0,0,0,0,0); //jan 1 2019
    REQUIRE(timestamp == 1546300800000 * 1000);
    
    timestamp = TimeUtils::build_time(2019,0,1,1,0,0,0,0); //jan 1 2019, 1:00:00
    REQUIRE(timestamp == 1546304400000 * 1000);    
    
    timestamp = TimeUtils::build_time(2019,0,1,1,42,0,0,0); //jan 1 2019, 1:42:00
    REQUIRE(timestamp == 1546306920000 * 1000);    
    
    timestamp = TimeUtils::build_time(2019,0,1,1,42,13,0,0); //jan 1 2019, 1:42:13
    REQUIRE(timestamp == 1546306933000 * 1000);        
    
    timestamp = TimeUtils::build_time(2019,0,1,1,42,13,5,0); //jan 1 2019, 1:42:13 + 5 ms
    REQUIRE(timestamp == 1546306933005 * 1000);        

    timestamp = TimeUtils::build_time(2019,0,1,1,42,13,5,6); //jan 1 2019, 1:42:13 + 5 ms  + 6 us
    REQUIRE(timestamp == 1546306933005 * 1000  + 6);    
}

TEST_CASE("test the build time with the parameters year, month, day and time in milliseconds")
{
    uint64_t timestamp = TimeUtils::build_time(1990,1,12,3661000);
    REQUIRE(TimeUtils::julianTime(timestamp)=="1990-44 1:1:1");
}

TEST_CASE("test the build time with the parameters year, year day, hour, minutes, time in microseconds")
{
    uint64_t timestamp = TimeUtils::build_time(1978,78,8,15,1000000);
    REQUIRE(TimeUtils::julianTime(timestamp)=="1978-79 8:15:1");
}

TEST_CASE("Julian time conversion test"){
    uint64_t timestamp = TimeUtils::build_time(2000,2,2,5,45,23,34,12);
    REQUIRE(TimeUtils::julianTime(timestamp)=="2000-62 5:45:23");    
}
*/
TEST_CASE("convertYearDay2Epoch") {
    uint64_t timestamp1970 = TimeUtils::convertCarisSvpDate2EpochMicro("1970-1 00:00:00");
    uint64_t timestamp2019 = TimeUtils::convertCarisSvpDate2EpochMicro("2019-1 00:00:00");
    uint64_t timestamp2020 = TimeUtils::convertCarisSvpDate2EpochMicro("2020-1 00:00:00");
    uint64_t timestampTimeOfWritingThisTest = TimeUtils::convertCarisSvpDate2EpochMicro("2019-231 15:30:21");
    
    std::string carisSvpTime = 
            std::to_string(2019) + "-" + 
            std::to_string(231) + " " + 
            std::to_string(15) + ":" + 
            std::to_string(30) + ":" + 
            std::to_string(21);
    
    uint64_t timestampCarisSvpTime = TimeUtils::convertCarisSvpDate2EpochMicro(carisSvpTime.c_str());
    
    //https://www.epochconverter.com/
    REQUIRE(timestamp1970 == 0l);
    REQUIRE(timestamp2019 == 1546300800000000);
    REQUIRE(timestamp2020 == 1577836800000000);
    REQUIRE(timestampTimeOfWritingThisTest == 1566228621000000);
    REQUIRE(timestampCarisSvpTime == 1566228621000000);
}