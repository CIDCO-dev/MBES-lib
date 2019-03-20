/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   SoundVelocityProfile.h
 * Author: glm,jordan
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

class SoundVelocityProfile {
public:

    SoundVelocityProfile();

    ~SoundVelocityProfile();


    void write(std::string & filename);

    unsigned int getSize() {
        return size;
    };

    double getLatitude() 	 { return latitude; }
    void   setLatitude(double l) { latitude=l;}

    double getLongitude()	 { return longitude;}
    void   setLongitude(double l){ longitude=l;}

    uint64_t getTimestamp(){ return microEpoch;};
    void     setTimestamp(uint64_t t) { microEpoch=t;};

    void     add(double depth,double soundSpeed);

    Eigen::VectorXd & getDepths();
    Eigen::VectorXd & getSpeeds();

private:
    unsigned int size;

    uint64_t  microEpoch; //timestamp

    double latitude;
    double longitude;

    Eigen::VectorXd depths;
    Eigen::VectorXd speeds;

    std::vector<std::pair<double,double>> samples;
};

SoundVelocityProfile::SoundVelocityProfile() {
    longitude = latitude = nan("");
};

SoundVelocityProfile::~SoundVelocityProfile() {
};

void SoundVelocityProfile::add(double depth,double soundSpeed){
	samples.push_back(std::make_pair(depth,soundSpeed));
}

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
