#ifndef TIMEUTILS_HPP
#define TIMEUTILS_HPP

#include <cstring>
#include "../SoundVelocityProfile.hpp"
#ifdef _WIN32
#include <ctime>
#endif
/**
 * Returns epoch in microseconds since Jan 1 1970
 */

uint64_t build_time(int year,int month,int day,int hour,int minutes,int seconds,int millis,int microseconds){
	struct tm t;
	memset(&t,0,sizeof(struct tm));

	t.tm_sec=seconds;
	t.tm_min=minutes;
	t.tm_hour=hour;
	t.tm_mday=day;
	t.tm_mon=month;
	t.tm_year=year - 1900;

	uint64_t res = mktime(&t)*1000000 + millis * 1000 + microseconds;

	return res;
}

/**
 * Returns epoch in microseconds since Jan 1 1970
 *
 * Time includes hours and minutes since midnight
 */
uint64_t build_time(int year,int month,int day,long timeInMilliseconds){
    struct tm t;
    memset(&t,0,sizeof(struct tm));

    t.tm_sec=0;
    t.tm_min=0;
    t.tm_hour=0;
    t.tm_mday=day;
    t.tm_mon=month;
    t.tm_year=year - 1900;

    uint64_t res = mktime(&t)*1000000 + timeInMilliseconds * 1000;

    return res;
}

/**
 * Returns epoch in microseconds since Jan 1 1970
 *
 * Time includes hours and minutes since midnight
 * yday : 0-365
 * hour: 0-23
 * minute: 0-59
 */
uint64_t build_time(int year,int yday, int hour, int minutes, long timeInMicroSeconds){
    struct tm t;
    memset(&t,0,sizeof(struct tm));

    t.tm_sec=0;
    t.tm_min=minutes;
    t.tm_hour=hour;
    t.tm_mday=yday; //hack around the C-standard: use "January 244th" since yday is an output parameter
    t.tm_year=year - 1900;

    uint64_t res = mktime(&t)*1000000 + timeInMicroSeconds;

    return res;
}

/**
 * Return the timestamp in julian time format (yyyy-ddd hh:mm:ss) 
 * 
 * @param mE number of microsecond of the timestamp 
 */
static std::string julianTime(uint64_t mE)
{
    time_t date = mE/1000000 + 18000;
    struct tm * timeinfo;
    timeinfo = localtime (&date);
    std::stringstream ssDate;
    ssDate << timeinfo->tm_year + 1900 << "-" << timeinfo->tm_yday + 1 << " " << timeinfo->tm_hour << ":" << timeinfo->tm_min << ":" << timeinfo->tm_sec;
    return ssDate.str();
}

#endif
