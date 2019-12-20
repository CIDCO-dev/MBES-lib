/*
 * Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

#ifndef TIMEUTILS_HPP
#define TIMEUTILS_HPP

#include <cstring>
#include <ctime>
#include <string>
#include <chrono>
#include <time.h>
#include <iomanip>
#include <sstream>
#include "Exception.hpp"

#ifdef _WIN32
#define timegm _mkgmtime
#endif

/*!
 * \brief TimeUtils class
 * \author Guillaume Labbe-Morissette
 */
class TimeUtils {
public:
    
    


    /**
     * Return the number microseconds since 1st January 1970 of the parameters in total
     *
     * @param year number of years (0-3000)
     * @param month number of months (0-11)
     * @param day number of days (1-31)
     * @param hour number of hours (0-23)
     * @param minutes number of minutes (0-59)
     * @param seconds number of seconds
     * @param millis number of milliseconds
     * @param microseconds number of microseconds
     */
    static uint64_t build_time(int year, int month, int day, int hour, int minutes, int seconds, int millis, int microseconds) {
        struct tm t;

        t.tm_year = year - 1900;
        t.tm_mon = month;
        t.tm_mday = day;
        t.tm_hour = hour;
        t.tm_min = minutes;
        t.tm_sec = seconds;

        time_t epochTime = timegm(&t);

        uint64_t microEpoch = (uint64_t) epochTime * 1000000 + (uint64_t) millis * 1000 + (uint64_t) microseconds;

        return microEpoch;
    }

    /**
     * Return the number microseconds since 1st January 1970 of the parameters in total
     * 
     * @param year number of year
     * @param month number of month less than an year
     * @param day number of day less than an month
     * @param timeInMilliseconds number of millisecond less than an day
     */
    static uint64_t build_time(int year, int month, int day, long timeInMilliseconds) {
        
        struct std::tm tm = {0};
        std::stringstream ssDate = convertDateTimeInfo2Stringstream(year, month, day, 0, 0, 0);
        ssDate >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
        
        time_t tTime = timegm(&tm);
        
        uint64_t epochMicro = 0;
        epochMicro = (uint64_t) tTime * 1000000 + timeInMilliseconds*1000;
        
        return epochMicro;
    }
    
    /**
     * Return the number microseconds since 1st January 1970 of the parameters in total
     *
     * @param year number of year
     * @param yday number of day less than an year
     * @param hour number of hour less than an day
     * @param minutes number of minute les than an hour
     * @param timeMicroseconds number of microsecond less than an minute
     */
    static uint64_t build_time(int year, int yday, int hour, int minutes, long timeInMicroSeconds) {
        
        int resultMonth, resultDayOfMonth;
        convertDayOfYear2YearMonthDay(year, yday, resultMonth, resultDayOfMonth);
        
        std::stringstream ssDate = convertDateTimeInfo2Stringstream(year, resultMonth, resultDayOfMonth, hour, minutes, 0);
        
        struct std::tm tm = {0};
        ssDate >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
        
        time_t tTime = timegm(&tm);
        
        uint64_t epochMicro = 0;
        epochMicro = (uint64_t) tTime * 1000000 + timeInMicroSeconds;
        
        return epochMicro;
    }
    
    static std::stringstream convertDateTimeInfo2Stringstream(int year, int month, int day, int hour, int minute, int second) {
        std::stringstream ssDate;
        
        ssDate << year << "-";
        
        if(month < 10) {
            ssDate << "0" << month << "-";
        } else {
            ssDate << month << "-";
        }
        
        if(day < 10) {
            ssDate << "0" << day << " ";
        } else {
            ssDate << day << " ";
        }
        
        if(hour < 10) {
            ssDate << "0" << hour << ":";
        } else {
            ssDate << hour << ":";
        }
        
        if(minute < 10) {
            ssDate << "0" << minute << ":";
        } else {
            ssDate << minute << ":";
        }
        
        if(second < 10) {
            ssDate << "0" << second;
        } else {
            ssDate << second;
        }
        
        return ssDate;
    }

    static uint64_t convertCarisSvpDate2EpochMicro(const char* carisTime) {
        
        int year;
        int yday;
        int hour;
        int minute;
        int second;
        
        if (std::sscanf(carisTime, "%d-%d %d:%d:%d", &year, &yday, &hour, &minute, &second) == 5) {
        } else {
            std::stringstream message;
            message << "Can't extract data from caris date string: " << carisTime;
            throw new Exception(message.str());
        }
        
        int month;
        int dayOfMonth;
        
        convertDayOfYear2YearMonthDay(year, yday, month, dayOfMonth);
        
        std::stringstream ssDate = convertDateTimeInfo2Stringstream(year, month, dayOfMonth, hour, minute, second);
        
        struct std::tm tmCaris = {0};
        ssDate >> std::get_time(&tmCaris, "%Y-%m-%d %H:%M:%S");
        
        
        
        time_t tTime = timegm(&tmCaris);

        return (uint64_t) tTime * 1000000; //seconds to microseconds
    }
    
    static bool extractJulianDateTimeInfo(const char* carisDateTime, int & year, int & yday, int & hour, int & minute, int & second) {
        if (std::sscanf(carisDateTime, "%d-%d %d:%d:%d", &year, &yday, &hour, &minute, &second) == 5) {
            return true;
        }
        
        return false;
    }
    
    static bool extractDateTimeInfo(const char* dateTime, int & year, int & month, int & day, int & hour, int & minute, int & second) {
        if (std::sscanf(dateTime, "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second) == 6) {
            return true;
        }
        
        return false;
    }

    /**
     * Return the timestamp in julian time format (yyyy-ddd hh:mm:ss)
     *
     * @param microEpoch number of microsecond of the timestamp
     */
    static std::string julianTime(uint64_t microEpoch) {
        time_t date = microEpoch / 1000000;
        struct tm * timeinfo;
        timeinfo = gmtime(&date);
        std::stringstream ssDate;
        ssDate << timeinfo->tm_year + 1900 << "-" << timeinfo->tm_yday + 1 << " " << timeinfo->tm_hour << ":" << timeinfo->tm_min << ":" << timeinfo->tm_sec;
        return ssDate.str();
    }
    
    /**
     * https://en.wikipedia.org/wiki/Leap_year#Algorithm
     * 
     * @param year the year
     */
    static bool isLeapYear(int year) {
        if(year % 4 == 0) {
            if(year % 100 == 0) {
                if(year % 400 == 0) {
                    return true;
                }
                
                return false;
            }
            
            return true;
        }
        
        return false;
    }
    
    /**
     * Convert year-month-day (YYYY-MM-DD) to julian time format (YYYY-jjj)
     *
     * @param year the year
     * @param month the month from 1 to 12
     * @param day the day of month from 1 to 31
     */
    static void convertYearMonthDay2DayOfYear(int year, int month, int day, int & resultYearDay) {
        
        if(month > 12) {
            std::stringstream message;
            message << "Month number is greater than 12: " << month;
            throw new Exception(message.str());
        } else if (day > 31) {
            std::stringstream message;
            message << "Day of month is greater than 31: " << day;
            throw new Exception(message.str());
        }
        
        int daysInMonths[12] = {
            31, // Jan
            28, // Feb (leap years handled below)
            31, // Mar
            30, // Apr
            31, // May
            30, // Jun
            31, // Jul
            31, // Aug
            30, // Sep
            31, // Oct
            30, // Nov
            31  // Dec
        };
        
        //Increment days in february for leap year
        if(isLeapYear(year)) {
            ++daysInMonths[1];
        }
        
        int day_of_year = 0;
        
        for(int i=1; i < month; i++) {
            day_of_year += daysInMonths[i-1];
        }
        
        resultYearDay = day_of_year + day;
    }

    /**
     * Convert julian time format (YYYY-jjj) to year-month-day (YYYY-MM-DD)
     *
     * @param year the year
     * @param yday the day of year
     */
    static void convertDayOfYear2YearMonthDay(int year, int yday, int & resultMonth, int & resultDayOfMonth) {
        
        if(yday > 366) {
            std::stringstream message;
            message << "Can't convert day of year to YYYY-MM-DD format since day number is greater than 366: " << yday;
            throw new Exception(message.str());
        } else if(yday == 366 && !isLeapYear(year)) {
            std::stringstream message;
            message << "Day number is 366 but not a leap year: " << year << "-" << yday;
            throw new Exception(message.str());
        }
        
        int daysInMonths[12] = {
            31, // Jan
            28, // Feb (leap years handled below)
            31, // Mar
            30, // Apr
            31, // May
            30, // Jun
            31, // Jul
            31, // Aug
            30, // Sep
            31, // Oct
            30, // Nov
            31  // Dec
        };
        
        //Increment days in february for leap year
        if(isLeapYear(year)) {
            ++daysInMonths[1];
        }
        
        int monthIndex = 0;
        int dayCounter = yday;
        
        while(dayCounter - daysInMonths[monthIndex] > 0 && monthIndex < 12) {
            dayCounter -= daysInMonths[monthIndex];
            ++monthIndex;
        }
        
        resultMonth = monthIndex+1;
        resultDayOfMonth = dayCounter;
    }

};
#endif
