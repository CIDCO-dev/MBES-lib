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
     * 
     * @param filename the name of the that will use to read
     */
    void read(std::string filename);
    
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
     */
    std::string julianTime()
    {
        time_t date = static_cast<time_t>(microEpoch/1000);
                struct tm * timeinfo;
                time (&date);
                timeinfo = localtime (&date);
                std::stringstream ssDate;
                ssDate << timeinfo->tm_year+1900 << "-" << timeinfo->tm_yday+1 << " " << timeinfo->tm_hour << ":" << timeinfo->tm_min << ":" << timeinfo->tm_sec;
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
     * Read a row of text who contains the timestamp of the song velocity profile 
     * and return it in microsecond
     * 
     * @param row the row who must be read
     */
    uint64_t readTime(std::string & row)
    {
        int year;
        int yday;
        int hour;
        int minute;
        int second;
        std::sscanf(row.c_str(),"%d%d%d%d%d",&year,&yday,&hour,&minute,&second);
        row.erase(0,row.find(" ")+1);
        row.erase(0,row.find(" ")+1);
        uint64_t nbrM = 0;
        nbrM = nbrM+year;
        nbrM = nbrM*365 + yday;
        nbrM = nbrM*24 + hour;
        nbrM = nbrM*60 + minute;
        nbrM = nbrM*60 + second;
        nbrM = nbrM*1000000;
        return nbrM;
    }
    
    /**
     * Read a row of text who contains the latitude or longitude of 
     * the song velocity  profile and return it in double
     * 
     * @param row the row who must be read 
     */
    double readLatLong(std::string & row)
    {
        std::string direction = row.substr(0, row.find(" "));
        row.erase(0,row.find(" ")+1);
        double degrees;
        double minute;
        double second;
        std::sscanf(row.c_str(), "%d%d%d",&degrees,&minute,&second);
        row.erase(0,row.find(" ")+1);
        double value = second/60 + minute;
        value = value/60 + degrees;
        if ((direction.compare("South") == 0)||(direction.compare("West")==0))
        {
            value = -value;
        }
        return value;
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

/**Create a sound velocity profile
     * 
     * @param timestamp timestamp of the sound velocity profile
     * @param platitude latitude of the sound velocity profile
     * @param plongitude longitude of the sound velocity profile
     */
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
	std::ofstream out(filename);

	if(out.is_open()){
		//TODO: write proper date and lat/lon
                std::string sDate;
                sDate = julianTime();
                std::string slat;
                slat = latFormat(latitude);
                std::string slong;
                slong = longFormat(longitude);
		out << "[SVP_VERSION_2]" << "\r\n";
		out << filename << "\r\n";
                out << "Section " << sDate << " " << slat << " " << slong << " \r\n" ;//FIXME: put date as yyyy-ddd hh:mm:ss dd:mm:ss (lat) dd:mm:ss (lon)
		for(unsigned int i=0;i<samples.size();i++){
			out << samples[i].first << " " << samples[i].second << " \r\n";
		}
		out.close();
	}
}

/**
     * Read a file who contains the information of a sound velocity profile
     * and change the values to fit with the information
     * 
     * @param filename the name of the that will use to read
     */
void SoundVelocityProfile::read(std::string filename)
{
    std::ifstream inFile;
    inFile.open(filename);
    std::string row;
    std::string cont;
    if(inFile)
    {
        int i = 0;
        while (inFile >> row)
        {
            if (i>1)
            {
                if (i==2)
                {
                    row.erase(0,8);
                    microEpoch = readTime(row);
                    latitude = readLatLong(row);
                    longitude = readLatLong(row);
                    samples = std::vector<std::pair<double,double>>();
                }
                else
                {
                    double deph = atof(row.substr(0,row.find(" ")).c_str());
                    row.erase(0,row.find(" ")+1);
                    double speed = atof(row.substr(0,row.find(" ")).c_str());
                    add(deph,speed);
                }
            }
            i = i+1;
        }
    }
    else
    {
       //Not sure how we should warn the user about the file not found
    }
    inFile.close();
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
