/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef TIMEUTILS_HPP
#define TIMEUTILS_HPP

#include <cstring>
#ifdef _WIN32
#include <ctime>
#endif

/*!
* \brief TimeUtils class
* \author Guillaume Labbe-Morissette
*/
class TimeUtils{
public:

/**
 * Returns epoch in microseconds since Jan 1 1970
 */

/**
 * Return the number microseconds since 1st January 1970 of the parameters in total
 * 
 * @param year number of year
 * @param month number of month less than an year
 * @param day number of day less than an month
 * @param hour number of hour less than an day
 * @param minutes number of minute les than an hour 
 * @param seconds number of second les than an second
 * @param millis number of millisecond less than an second
 * @param microseconds number of microsecond less than an millisecond
 */
static uint64_t build_time(int year,int month,int day,int hour,int minutes,int seconds,int millis,int microseconds){
        uint64_t nbrM = 0;
        year = year-1970;
        nbrM = nbrM+year;
        int m = month;
        int yday = 0;
        while (m>0)
        {
            switch(m)
            {
                case 11:
                 yday=yday+30;   
                break;
                
                case 10:
                 yday=yday+31;   
                break;
                
                case 9:
                 yday=yday+30;   
                break;
                
                case 8:
                 yday=yday+31;   
                break;
                
                case 7:
                 yday=yday+31;   
                break;
                
                case 6:
                 yday=yday+30;   
                break;
                
                case 5:
                 yday=yday+31;   
                break;
                
                case 4:
                 yday=yday+30;   
                break;
                
                case 3:
                 yday=yday+31;   
                break;
                
                case 2:
                 if (year % 4 == 0)
                 {
                     yday=yday+29;
                 }
                 else
                 {
                     yday=yday+28;
                 }
                break;
                
                case 1:
                 yday=yday+31;   
                break;
            }
            m = m-1;
        }
        yday=yday+day;
        nbrM = nbrM*365 + yday;
        int y = year+2;
        while (y >= 4)
        {
            y = y-4;
            nbrM = nbrM+1;
        }
        nbrM = nbrM*24 + hour;
        nbrM = nbrM*60 + minutes;
        nbrM = nbrM*60 + seconds;
        nbrM = nbrM*1000000 + millis * 1000 + microseconds;
	return nbrM;
}

/**
 * Return the number microseconds since 1st January 1970 of the parameters in total
 * 
 * @param year number of year
 * @param month number of month less than an year
 * @param day number of day less than an month
 * @param timeInMilliseconds number of millisecond less than an day
 */
static uint64_t build_time(int year,int month,int day,long timeInMilliseconds){
uint64_t nbrM = 0;
        year = year-1970;
        nbrM = nbrM+year;
        int m = month;
        int yday = 0;
        while (m>0)
        {
            switch(m)
            {
                case 11:
                 yday=yday+30;   
                break;
                
                case 10:
                 yday=yday+31;   
                break;
                
                case 9:
                 yday=yday+30;   
                break;
                
                case 8:
                 yday=yday+31;   
                break;
                
                case 7:
                 yday=yday+31;   
                break;
                
                case 6:
                 yday=yday+30;   
                break;
                
                case 5:
                 yday=yday+31;   
                break;
                
                case 4:
                 yday=yday+30;   
                break;
                
                case 3:
                 yday=yday+31;   
                break;
                
                case 2:
                 if (year % 4 == 0)
                 {
                     yday=yday+29;
                 }
                 else
                 {
                     yday=yday+28;
                 }
                break;
                
                case 1:
                 yday=yday+31;   
                break;
            }
            m = m-1;
        }
        yday=yday+day;
        nbrM = nbrM*365 + yday;
        int y = year+2;
        while (y >= 4)
        {
            y = y-4;
            nbrM = nbrM+1;
        }
        nbrM = nbrM*24*60*60*1000000 + timeInMilliseconds * 1000;
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
static uint64_t build_time(int year,int yday, int hour, int minutes, long timeInMicroSeconds){
    uint64_t nbrM = 0;
        year = year-1970;
        nbrM = nbrM+year;
        nbrM = nbrM*365 + yday;
        int y = year+2;
        while (y >= 4)
        {
            y = y-4;
            nbrM = nbrM+1;
        }
        nbrM = nbrM*24 + hour;
        nbrM = nbrM*60 + minutes;
        nbrM = nbrM*60*1000000 + timeInMicroSeconds;
	return nbrM;
}

/**
 * Return the timestamp in julian time format (yyyy-ddd hh:mm:ss) 
 * 
 * @param microEpoch number of microsecond of the timestamp 
 */
static std::string julianTime(uint64_t microEpoch)
{
    time_t date = microEpoch/1000000;
    struct tm * timeinfo;
    timeinfo = gmtime(&date);
    std::stringstream ssDate;
    ssDate << timeinfo->tm_year + 1900 << "-" << timeinfo->tm_yday + 1 << " " << timeinfo->tm_hour << ":" << timeinfo->tm_min << ":" << timeinfo->tm_sec;
    return ssDate.str();
}

};
#endif
