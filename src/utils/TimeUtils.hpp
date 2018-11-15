#ifndef TIMEUTILS_HPP
#define TIMEUTILS_HPP

#include <cstring>
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

#endif
