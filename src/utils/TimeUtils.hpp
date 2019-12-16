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
     * Returns epoch in microseconds since Jan 1 1970
     */

#ifdef _WIN32

    static char *strptime(const char * s, const char* f, struct tm* tm) {
        std::istringstream input(s);
        input.imbue(std::locale(setlocale(LC_ALL, nullptr)));
        input >> std::get_time(tm, f);
        if (input.fail()) {
            return nullptr;
        }
        return (char*) (s + input.tellg());
    }
#endif

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
     * << std::endl
     * @param year number of year
     * @param month number of month less than an year
     * @param day number of day less than an month
     * @param timeInMilliseconds number of millisecond less than an day
     */
    static uint64_t build_time(int year, int month, int day, long timeInMilliseconds) {
        uint64_t nbrM = 0;
        year = year - 1970;
        nbrM = nbrM + year;
        int m = month;
        int yday = 0;
        while (m > 0) {
            switch (m) {
                case 11:
                    yday = yday + 30;
                    break;

                case 10:
                    yday = yday + 31;
                    break;

                case 9:
                    yday = yday + 30;
                    break;

                case 8:
                    yday = yday + 31;
                    break;

                case 7:
                    yday = yday + 31;
                    break;

                case 6:
                    yday = yday + 30;
                    break;

                case 5:
                    yday = yday + 31;
                    break;

                case 4:
                    yday = yday + 30;
                    break;

                case 3:
                    yday = yday + 31;
                    break;

                case 2:
                    if (year % 4 == 0) {
                        yday = yday + 29;
                    } else {
                        yday = yday + 28;
                    }
                    break;

                case 1:
                    yday = yday + 31;
                    break;
            }
            m = m - 1;
        }
        yday = yday + day;
        nbrM = nbrM * 365 + yday;
        int y = year + 2;
        while (y >= 4) {
            y = y - 4;
            nbrM = nbrM + 1;
        }
        nbrM = nbrM * 24 * 60 * 60 * 1000000 + timeInMilliseconds * 1000;
        return nbrM;
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
        uint64_t nbrM = 0;
        year = year - 1970;
        nbrM = nbrM + year;
        nbrM = nbrM * 365 + yday;
        int y = year + 2;
        while (y >= 4) {
            y = y - 4;
            nbrM = nbrM + 1;
        }
        nbrM = nbrM * 24 + hour;
        nbrM = nbrM * 60 + minutes;
        nbrM = nbrM * 60 * 1000000 + timeInMicroSeconds;
        return nbrM;
    }

    static uint64_t convertCarisSvpDate2EpochMicro(const char* carisTime) {
        struct tm tmTime;
        strptime(carisTime, "%Y-%j %H:%M:%S", &tmTime);
        time_t tTime = timegm(&tmTime);

        return (uint64_t) tTime * 1000000; //seconds to microseconds
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
     * Convert julian time format (YYYY-jjj) to year-month-day (YYYY-MM-DD)
     *
     * @param year the year
     * @param yday the day of year
     */
    static std::string convertDayOfYear2YearMonthDay(int year, int yday) {
        
        if(yday > 366) {
            std::stringstream message;
            message << "Can't convert day of year to YYYY-MM-DD format since day number is greater than 366: " << yday;
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
        if(year % 4 == 0) {
            if(year % 100 == 0) {
                if(year % 400 == 0) {
                    ++daysInMonths[1];
                }
            } else {
                ++daysInMonths[1];
            }
        }
        
        int monthIndex = 0;
        int dayCounter = yday;
        
        while(dayCounter - daysInMonths[monthIndex] > 0 && monthIndex < 12) {
            daycounter -= daysInMonths[monthIndex];
            ++monthIndex;
        }
        
        std::stringstream ssDate;
        
        ssDate << year << "-";
        
        if(monthIndex < 9) {
            ssDate << "0" << monthIndex+1 << "-";
        } else {
            ssDate << monthIndex+1 << "-";
        }
        
        if(dayCounter < 10) {
            ssDate << "0" << dayCounter;
        } else {
            ssDate << dayCounter;
        }
        
        return ssDate.str();
    }

};
#endif
