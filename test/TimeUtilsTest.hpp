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

#ifdef _WIN32
#define timegm _mkgmtime
#endif

/*
Alhtough the following commented tests pass, the build_time functions are wrong!

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

TEST_CASE("extract time from string test") {
    std::string timeJulian = "1970-235 14:37:49";

    int year;
    int yday;
    int hour;
    int minute;
    int second;
    
    bool extractionSuccess = TimeUtils::extractJulianDateTimeInfo(timeJulian.c_str(), year, yday, hour, minute, second);
    
    REQUIRE(extractionSuccess);
    REQUIRE(year == 1970);
    REQUIRE(yday == 235);
    REQUIRE(hour == 14);
    REQUIRE(minute == 37);
    REQUIRE(second == 49);

}

TEST_CASE("Convert standard date time to epoch micro test") {
    std::string time1 = "1970-01-01 00:00:00"; // 0
    std::string time2 = "2019-12-17 08:39:39"; // 1576615179000000

    std::stringstream ssDate1(time1);
    std::stringstream ssDate2(time2);

    struct std::tm tm1 = {0};
    struct std::tm tm2 = {0};
    
    ssDate1 >> std::get_time(&tm1, "%Y-%m-%d %H:%M:%S");
    ssDate2 >> std::get_time(&tm2, "%Y-%m-%d %H:%M:%S");
    
    time_t epochTime1 = timegm(&tm1);
    time_t epochTime2 = timegm(&tm2);
    
    REQUIRE((uint64_t) epochTime1 * 1000000 == 0l);
    REQUIRE((uint64_t) epochTime2 * 1000000 == 1576571979000000l);
}

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

TEST_CASE("Leap year test") {
    
    bool notLeapYear1900 = !TimeUtils::isLeapYear(1900);
    bool notLeapYear1999 = !TimeUtils::isLeapYear(1999);
    bool leapYear2000 = TimeUtils::isLeapYear(2000);
    bool leapYear2016 = TimeUtils::isLeapYear(2016);
    
    REQUIRE(notLeapYear1900);
    REQUIRE(notLeapYear1999);
    REQUIRE(leapYear2000);
    REQUIRE(leapYear2016);
}

TEST_CASE("Conversion between YYYY-jjj and YYYY-MM-DD for January 1st 1970") {
    int year = 1970;
    int month = 1;
    int day = 1;
    int yday = 1;
    
    int resultMonth, resultDayOfMonth;
    TimeUtils::convertDayOfYear2YearMonthDay(year, yday, resultMonth, resultDayOfMonth);
    
    REQUIRE(resultMonth == month);
    REQUIRE(resultDayOfMonth == day);
    
    int resultYearDay;
    
    TimeUtils::convertYearMonthDay2DayOfYear(year, month, day, resultYearDay);
    
    REQUIRE(resultYearDay == yday);
}

TEST_CASE("Conversion between YYYY-jjj and YYYY-MM-DD for December 31st 1970") {
    int year = 1970;
    int month = 12;
    int day = 31;
    int yday = 365;
    
    int resultMonth, resultDayOfMonth;
    
    TimeUtils::convertDayOfYear2YearMonthDay(year, yday, resultMonth, resultDayOfMonth);
    
    REQUIRE(resultMonth == month);
    REQUIRE(resultDayOfMonth == day);
    
    int resultYearDay;
    
    TimeUtils::convertYearMonthDay2DayOfYear(year, month, day, resultYearDay);
    
    REQUIRE(resultYearDay == yday);
}

TEST_CASE("Conversion between YYYY-jjj and YYYY-MM-DD for January 1st 2000") {
    int year = 2000;
    int month = 1;
    int day = 1;
    int yday = 1;
    
    int resultMonth, resultDayOfMonth;
    
    TimeUtils::convertDayOfYear2YearMonthDay(year, yday, resultMonth, resultDayOfMonth);
    
    REQUIRE(resultMonth == month);
    REQUIRE(resultDayOfMonth == day);
    
    int resultYearDay;
    
    TimeUtils::convertYearMonthDay2DayOfYear(year, month, day, resultYearDay);
    
    REQUIRE(resultYearDay == yday);
}

TEST_CASE("Conversion between YYYY-jjj and YYYY-MM-DD for December 31st 2000") {
    int year = 2000;
    int month = 12;
    int day = 31;
    int yday = 366;
    
    int resultMonth, resultDayOfMonth;
    
    TimeUtils::convertDayOfYear2YearMonthDay(year, yday, resultMonth, resultDayOfMonth);
    
    REQUIRE(resultMonth == month);
    REQUIRE(resultDayOfMonth == day);
    
    int resultYearDay;
    
    TimeUtils::convertYearMonthDay2DayOfYear(year, month, day, resultYearDay);
    
    REQUIRE(resultYearDay == yday);
}

TEST_CASE("Conversion between YYYY-jjj and YYYY-MM-DD for January 1st 2016") {
    int year = 2016;
    int month = 1;
    int day = 1;
    int yday = 1;
    
    int resultMonth, resultDayOfMonth;
    
    TimeUtils::convertDayOfYear2YearMonthDay(year, yday, resultMonth, resultDayOfMonth);
    
    REQUIRE(resultMonth == month);
    REQUIRE(resultDayOfMonth == day);
    
    int resultYearDay;
    
    TimeUtils::convertYearMonthDay2DayOfYear(year, month, day, resultYearDay);
    
    REQUIRE(resultYearDay == yday);
}

TEST_CASE("Conversion between YYYY-jjj and YYYY-MM-DD for December 31st 2016") {
    int year = 2016;
    int month = 12;
    int day = 31;
    int yday = 366;
    
    int resultMonth, resultDayOfMonth;
    
    TimeUtils::convertDayOfYear2YearMonthDay(year, yday, resultMonth, resultDayOfMonth);
    
    REQUIRE(resultMonth == month);
    REQUIRE(resultDayOfMonth == day);
    
    int resultYearDay;
    
    TimeUtils::convertYearMonthDay2DayOfYear(year, month, day, resultYearDay);
    
    REQUIRE(resultYearDay == yday);
}