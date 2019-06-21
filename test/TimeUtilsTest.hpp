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