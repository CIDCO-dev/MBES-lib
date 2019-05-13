/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
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
    uint64_t timestamp = TimeUtils::build_time(2000,2,2,5,45,23,34,12);
    REQUIRE(TimeUtils::julianTime(timestamp)=="2000-63 5:45:23");
    
    timestamp = TimeUtils::build_time(1970,0,0,0,0,0,34,12);
    REQUIRE(timestamp == 34012);
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