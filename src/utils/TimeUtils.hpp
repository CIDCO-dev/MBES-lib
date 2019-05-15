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
	* Returns the number in microseconds since 1st January 1970 of the parameters in total
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
	* Returns the number in microseconds since 1st January 1970 of the parameters in total
	*
	* @param year number of year
	* @param month number of month less than an year
	* @param day number of day less than an month
	* @param timeInMilliseconds number of millisecond less than an day
	*/
	static uint64_t build_time(int year,int month,int day,long timeInMilliseconds){
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
	* Returns the number in microseconds since 1st January 1970 of the parameters in total
	*
	* @param year number of year
	* @param yday number of day less than an year
	* @param hour number of hour less than an day
	* @param minutes number of minute les than an hour
	* @param timeMicroseconds number of microsecond less than an minute
	*/
	static uint64_t build_time(int year,int yday, int hour, int minutes, long timeInMicroSeconds){
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
	* Returns the timestamp in julian time format (yyyy-ddd hh:mm:ss)
	*
	* @param microEpoch number of microsecond of the timestamp
	*/
	static std::string julianTime(uint64_t microEpoch)
	{
		time_t date = microEpoch/1000000 + 18000;
		struct tm * timeinfo;
		timeinfo = localtime (&date);
		std::stringstream ssDate;
		ssDate << timeinfo->tm_year + 1900 << "-" << timeinfo->tm_yday + 1 << " " << timeinfo->tm_hour << ":" << timeinfo->tm_min << ":" << timeinfo->tm_sec;
		return ssDate.str();
	}

};
#endif
