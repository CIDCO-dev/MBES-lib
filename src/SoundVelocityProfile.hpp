/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   SoundVelocityProfile.h
 * Author: glm,jordan, emilegagne
 *
 * Created on August 22, 2018, 10:30 AM
 */

#ifndef SOUNDVELOCITYPROFILE_HPP
#define SOUNDVELOCITYPROFILE_HPP

#include <list>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>
#include <ctime>
#include <string>
#include "../src/utils/TimeUtils.hpp"

/*!
 * \brief Sound velocity profile class
 */
class SoundVelocityProfile {
public:

    /**Create a sound velocity*/
    SoundVelocityProfile();

    /**Destroy a sound velocity*/
    ~SoundVelocityProfile();


    /**
     * Write the informations of the sound velocity profile in a file
     * 
     * @param filename the name of the file that will be use to write
     */
    void write(std::string & filename); 
    
    /**
     * Read a file who contains the information of a sound velocity profile
     * return true if the reading has been made successfully
     * 
     * @param filename the name of the that will use to read
     */
    bool read(std::string filename);
    
    /**Return the size of the sound velocity profile*/
    unsigned int getSize() {
        return size;
    };

    /**Return the latitude of the sound velocity profile*/
    double getLatitude() 	 { return latitude; }
    
    /**
     * Change the latitude of the sound velocity profile
     * 
     * @param l the new latitude
     */
    void   setLatitude(double l) { latitude=l;}

    /**Return the longitude of the sound velocity*/
    double getLongitude()	 { return longitude;}
    
    /**
     * Change the longitude of the sound velocity profile
     * 
     * @param l the new longitude of the sound velocity profile
     */
    void   setLongitude(double l){ longitude=l;}

    /**Return the timestamp of the sound velocity*/
    uint64_t getTimestamp(){ return microEpoch;};
    
    /**
     * Change the timestamp of the sound velocity profile
     * 
     * @param t the new timestamp
     */
    void     setTimestamp(uint64_t t) { microEpoch=t;};
    
    /**
     * Return the timestamp in julian time format (yyyy-ddd hh:mm:ss) 
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
    
    /**
     * Return the latitude in text form
     * 
     * @param value the latitude
     */
    std::string latFormat(double value)
    {
        std::string direction;
        if (value<0)
        {
            direction = "South";
        }
        else
        {
            direction = "North";
        }
        return latlongFormat(value,direction);
    }
    
    /**
     * Return the longitude in text form
     * 
     * @param value the longitude
     */
    std::string longFormat(double value)
    {
        std::string direction;
        if (value<0)
        {
            direction = "West";
        }
        else
        {
            direction = "East";
        }
        return latlongFormat(value,direction);
    }
    
    /**
     * Return the latitude or the longitude in format dd:mm:ss
     * 
     * @param value the latitude or longitude to convert
     * @param direction direction(N,W,E,S) of the latitude or longitude
     */
    std::string latlongFormat(double value, std::string direction)
    {
        std::stringstream ss;
                value = std::abs(value);
                double degrees = std::trunc(value);
                value = (value - degrees) * 60;
                double minutes = std::trunc(value);
                double second = (value - minutes) * 60;
                ss << direction << " " << degrees << ":" << minutes << ":" << second;
                return ss.str();
    }

    /**
     * Read a row of text who contains the timestamp, the latitude and longitude of 
     * the song velocity  profile and return true if the reading has been made successfully
     * 
     * @param row the row who must be read 
     * @param nbrM value of the timestamp we get after the reading
     * @param lat value of the latitude we get after the reading
     * @param lon value of the longitude we get after the reading
     */
    bool readTimeLatLong(std::string & row, uint64_t &nbrM,double &lat, double &lon)
    {
        char latdirection[5];
        double latdegrees;
        double latminute;
        double latsecond;
        char londirection[4];
        double londegrees;
        double lonminute;
        double lonsecond;
        int year;
        int yday;
        int hour;
        int minute;
        int second;
        if (std::sscanf(row.c_str(), "Section %d-%d %d:%d:%d %5s %lf:%lf:%lf %4s %lf:%lf:%lf",
                &year,&yday,&hour,&minute,&second,latdirection,&latdegrees,&latminute,&latsecond,
                londirection,&londegrees,&lonminute,&lonsecond)==13)
        {
            year = year-1970;
            yday = yday-1;
            nbrM = nbrM+year;
            nbrM = nbrM*365 + yday;
            nbrM = nbrM*24 + hour;
            nbrM = nbrM*60 + minute;
            nbrM = nbrM*60 + second;
            nbrM = nbrM*1000000;
            lat = latsecond/60 + latminute;
            lat = lat/60 + latdegrees;
            std::string sdirection;
            sdirection = latdirection;
            if (sdirection.compare("South") == 0)
            {
                lat = -lat;
            }
            else if (sdirection.compare("North")!=0)
            {
                return false;
            }
            lon = lonsecond/60 + lonminute;
            lon = lon/60 + londegrees;
            sdirection = londirection;
            if (sdirection.compare("West") == 0)
            {
                lon = -lon;
            }
            else if (sdirection.compare("East")!=0)
            {
                return false;
            }
            return true;
        }
        else
        {
            return false;   
        }
    }
    
    /**
     * Add a new value in the vector depths and speeds
     * 
     * @param depth value to add in depths
     * @param soundSpeed value to add in speeds
     */
    void     add(double depth,double soundSpeed);

    /**Return the vector depths*/
    Eigen::VectorXd & getDepths();
    
    /**Return the vector speeds*/
    Eigen::VectorXd & getSpeeds();

private:
    
    /**value of the sound velocity profile size*/
    unsigned int size;

    /**timestamp value of the sound velocity profile (micro-second)*/
    uint64_t  microEpoch; //timestamp

    /**latitude value of the sound velocity profile*/
    double latitude;
    
    /**longitude value of the sound velocity profile*/
    double longitude;

    /**vector who contain the dephts of the sound velocity profile*/
    Eigen::VectorXd depths;
    
    /**vector who contain the speeds of the sound velocity profile*/
    Eigen::VectorXd speeds;

    /**vector who contain the depths and the speeds*/
    std::vector<std::pair<double,double>> samples;
};

    /**Create a sound velocity profile*/
SoundVelocityProfile::SoundVelocityProfile() {
    longitude = latitude = nan("");
};

/**Destroy the sound velocity*/
SoundVelocityProfile::~SoundVelocityProfile() {
};

     /**
     * Add a new value in the vector depths and speeds
     * 
     * @param depth value to add in depths
     * @param soundSpeed value to add in speeds
     */
void SoundVelocityProfile::add (double depth,double soundSpeed){
	samples.push_back(std::make_pair(depth,soundSpeed));
}

     /**
     * Write the information of the sound velocity profile in a file
     * 
     * @param filename the name of the file that will be use to write
     */
void SoundVelocityProfile::write(std::string & filename){
	std::ofstream out;
        out.open(filename);
	if(out.is_open()){
		//TODO: write proper date and lat/lon
                std::string sDate;
                sDate = julianTime(microEpoch);
                std::string slat;
                slat = latFormat(latitude);
                std::string slong;
                slong = longFormat(longitude);
		out << "[SVP_VERSION_2]" << "\r\n";
		out << filename << " \r\n";
                out << "Section " << sDate << " " << slat << " " << slong << " \r\n" ;//FIXME: put date as yyyy-ddd hh:mm:ss dd:mm:ss (lat) dd:mm:ss (lon)
		for(unsigned int i=0;i<samples.size();i++){
			out << samples[i].first << " " << samples[i].second << " \r\n";
		}
		out.close();
	}
}

/**
     * Read a file who contains the information of a sound velocity profile
     * return true if the reading has been made successfully
     * 
     * @param filename the name of the that will use to read
     */
bool SoundVelocityProfile::read(std::string filename)
{
    std::ifstream inFile;
    inFile.open(filename);
    std::string row;
    std::string cont;
    bool valide = true;
    if(inFile)
    {
        int i = 0;
        while ((std::getline(inFile,row))&&(valide))
        {
            if (i == 0)
            {
                int ver;
                if (std::sscanf(row.c_str(),"[SVP_VERSION_%d]",&ver)!=1)
                {
                    valide = false;
                }
            }
            else if (i==1)
            {
                std::string name;
                name = row.substr(0,row.find(" "));
                if(name.compare(filename)!=0)
                {
                    valide = false;
                }
            }
            else if (i==2)
            {
                uint64_t ms = 0;
                double lat = 0;
                double lon = 0;
                if(readTimeLatLong(row,ms,lat,lon))
                {
                    microEpoch = ms;
                    latitude = lat;
                    longitude = lon;
                }
                else
                {
                    valide = false;
                }
                samples = std::vector<std::pair<double,double>>();
            }
            else
            {
                double deph;
                double speed;
                if (std::sscanf(row.c_str(), "%lf %lf",&deph,&speed)==2)
                {
                    add(deph,speed);
                }
                else
                {
                    valide = false;
                }
            }
            i = i+1;
        }
    }
    else
    {
        valide = false;
    }
    inFile.close();
    return valide;
}

/**Return the vector depths*/
Eigen::VectorXd & SoundVelocityProfile::getDepths(){
	//lazy load internal vector
	if((unsigned int)depths.size() != samples.size()){
		depths.resize(samples.size());

		for(unsigned int i = 0;i<samples.size();i++){
			depths(i)=samples[i].first;
		}
	}

	return depths;
}

/**Return the vector speeds*/
Eigen::VectorXd & SoundVelocityProfile::getSpeeds(){
	//lazy load internal vector
	if((unsigned int)speeds.size() != samples.size()){
		speeds.resize(samples.size());

		for(unsigned int i=0;i<samples.size();i++){
			speeds(i)=samples[i].second;
		}
	}

	return speeds;
}

#endif /* SOUNDVELOCITYPROFILE_HPP */
