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

/*!
 * \brief Sound velocity class
 */
class SoundVelocityProfile {
public:

    /**Create a velocity*/
    SoundVelocityProfile();

    /**Destroy a velocity*/
    ~SoundVelocityProfile();


    /**
     * Write the informations of the velocity in a file
     * 
     * @param filename the name of the file that will be use to write
     */
    void write(std::string & filename);

    /**Return the size of the velocity*/
    unsigned int getSize() {
        return size;
    };

    /**Return the latitude of the velocity*/
    double getLatitude() 	 { return latitude; }
    
    /**
     * Change the latitude of the velocity
     * 
     * @param l the new latitude
     */
    void   setLatitude(double l) { latitude=l;}

    /**Return the longitude of the velocity*/
    double getLongitude()	 { return longitude;}
    
    /**
     * Change the longitude of the velocity
     * 
     * @param l the new longitude of the velocity
     */
    void   setLongitude(double l){ longitude=l;}

    /**Return the timestamp of the velocity*/
    uint64_t getTimestamp(){ return microEpoch;};
    
    /**
     * Change the timestamp of the velocity
     * 
     * @param t the new timestamp
     */
    void     setTimestamp(uint64_t t) { microEpoch=t;};
    
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
    
    /**value of the velocity size*/
    unsigned int size;

    /**timestamp value of the velocity (micro-seconde)*/
    uint64_t  microEpoch; //timestamp

    /**latitude value of the velocity*/
    double latitude;
    
    /**longitude value of the velocity*/
    double longitude;

    /**vector who contain the dephts of the velocity*/
    Eigen::VectorXd depths;
    
    /**vector who contain the speeds of the velocity*/
    Eigen::VectorXd speeds;

    /**vector who contain the depths and the speeds*/
    std::vector<std::pair<double,double>> samples;
};

/**Create the velocity*/
SoundVelocityProfile::SoundVelocityProfile() {
    longitude = latitude = nan("");
};

/**Destroy the velocity*/
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
     * Write the informations of the velocity in a file
     * 
     * @param filename the name of the file that will be use to write
     */
void SoundVelocityProfile::write(std::string & filename){
	std::ofstream out(filename);

	if(out.is_open()){
		//TODO: write proper date and lat/lon
		out << "[SVP_VERSION_2]" << "\r\n";
		out << filename << "\r\n";
		out << "Section 2000-307 18:59:00 43:04:40 -070:42:42 This time and coordinates are wrong" << "\r\n"; //FIXME: put date as yyyy-ddd hh:mm:ss dd:mm:ss (lat) dd:mm:ss (lon)

		for(auto i=samples.begin();i!=samples.end();i++){
			out << (*i).first << " " << (*i).second << "\r\n";
		}

		out.close();
	}
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
