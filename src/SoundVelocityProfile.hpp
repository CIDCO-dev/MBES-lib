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

/*!
 * \brief Sound velocity class
 */
class SoundVelocityProfile {
public:

    /**Create a sound velocity
     * 
     * @param timestamp timestamp of the sound velocity
     * @param platitude latitude of the sound velocity
     * @param plongitude longitude of the sound velocity
     */
    SoundVelocityProfile(uint64_t timestamp, double platitude, double plongitude);

    /**Destroy a sound velocity*/
    ~SoundVelocityProfile();


    /**
     * Write the informations of the sound velocity in a file
     * 
     * @param filename the name of the file that will be use to write
     */
    void write(std::string & filename); 
    
    /**
     * Read a file who contains the informations of a sound velocity
     * 
     * @param filename the name of the that will use to read
     */
    std::string read(std::string filename);
    
    /**
     * Return the timestamp in julian time format (yyyy-ddd hh:mm:ss) 
     */
    std::stringstream julianTime();
    
    /**
     * Return the latitude or the longitude in format dd:mm:ss
     * 
     * @param value the latitude or longitude to convert
     */
    std::stringstream latlongFormat(double value);

    /**Return the size of the sound velocity*/
    unsigned int getSize() {
        return size;
    };

    /**Return the latitude of the sound velocity*/
    double getLatitude() 	 { return latitude; }
    
    /**
     * Change the latitude of the sound velocity
     * 
     * @param l the new latitude
     */
    void   setLatitude(double l) { latitude=l;}

    /**Return the longitude of the sound velocity*/
    double getLongitude()	 { return longitude;}
    
    /**
     * Change the longitude of the sound velocity
     * 
     * @param l the new longitude of the sound velocity
     */
    void   setLongitude(double l){ longitude=l;}

    /**Return the timestamp of the sound velocity*/
    uint64_t getTimestamp(){ return microEpoch;};
    
    /**
     * Change the timestamp of the sound velocity
     * 
     * @param t the new timestamp
     */
    void     setTimestamp(uint64_t t) { microEpoch=t;};
    
    /**
     * Return the timestamp in julian time format (yyyy-ddd hh:mm:ss) 
     */
    std::stringstream julianTime()
    {
        time_t date = time(microEpoch);
                struct tm * timeinfo;
                time (&date)
                timeinfo = localtime (&date);
                std::stringstream ssDate;
                ssDate << timeinfo->tm_year+1990 << "-" << timeinfo->tm_yday+1 << " " << timeinfo->tm_hour << ":" << timeinfo->tm_min << ":" << timeinfo->tm_sec;
                return ssDate;
    }
    
    /**
     * Return the latitude or the longitude in format dd:mm:ss
     * 
     * @param value the latitude or longitude to convert
     */
    std::stringstream latlongFormat(double value)
    {
        std::stringstream ss;
                value = std::abs(value);
                double degrees = std::trunc(value);
                value = (value - degrees) * 60;
                double minutes = std::trunc(value);
                double second = (value - minutes) * 60;
                ss << degrees << ":" << minutes << ":" << second;
                return ss;
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
    
    /**value of the sound velocity size*/
    unsigned int size;

    /**timestamp value of the sound velocity (micro-second)*/
    uint64_t  microEpoch; //timestamp

    /**latitude value of the sound velocity*/
    double latitude;
    
    /**longitude value of the sound velocity*/
    double longitude;

    /**vector who contain the dephts of the sound velocity*/
    Eigen::VectorXd depths;
    
    /**vector who contain the speeds of the sound velocity*/
    Eigen::VectorXd speeds;

    /**vector who contain the depths and the speeds*/
    std::vector<std::pair<double,double>> samples;
};

/**Create a sound velocity
     * 
     * @param timestamp timestamp of the sound velocity
     * @param platitude latitude of the sound velocity
     * @param plongitude longitude of the sound velocity
     */
SoundVelocityProfile::SoundVelocityProfile(uint64_t timestamp, double platitude, double plongitude) {
    microEpoch = timestamp;
    latitude = platitude;
    longitude = plongitude;
    //longitude = latitude = nan("");
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
void SoundVelocityProfile::add(double depth,double soundSpeed){
	samples.push_back(std::make_pair(depth,soundSpeed));
}

     /**
     * Write the informations of the sound velocity in a file
     * 
     * @param filename the name of the file that will be use to write
     */
void SoundVelocityProfile::write(std::string & filename){
	std::ofstream out(filename);

	if(out.is_open()){
		//TODO: write proper date and lat/lon
                std::stringstream ssDate;
                ssDate = julianTime();
                std::stringstream sslat;
                sslat = latlongFormat(latitude);
                std::stringstream sslong;
                sslong = latlongFormat(longitude);
		out << "[SVP_VERSION_2]" << "\r\n";
		out << filename << "\r\n";
                out << "Section " << ssDate.str() << " " << sslat.str() << " " << sslong.str() << "\r\n" ;//FIXME: put date as yyyy-ddd hh:mm:ss dd:mm:ss (lat) dd:mm:ss (lon)
		for(unsigned int i=0;i<samples.size();i++){
			out << samples[i].first << " " << samples[i].second << "\r\n";
		}
		out.close();
	}
}

/**
     * Read a file who contains the informations of a sound velocity
     * 
     * @param filename the name of the that will use to read
     */
std::string read(std::string filename)
{
    std::ifstream inFile;
    inFile.open(filename);
    std::string row;
    std::string cont;
    if(inFile)
    {
        while (inFile >> row)
        {
            cont = cont + row;
        }
    }
    else
    {
        cont = "Error: file not found";
    }
    inFile.close();
    return cont;
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
