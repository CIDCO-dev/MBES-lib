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

    unsigned int getSize() {
        return size;
    };

    double getLatitude() 	 { return latitude; }
    void   setLatitude(double l) { latitude=l;}

    double getLongitude()	 { return longitude;}
    void   setLongitude(double l){ longitude=l;}

    void   add(double depth,double soundSpeed);

private:
    unsigned int size;

    double latitude;
    double longitude;

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

#endif /* SOUNDVELOCITYPROFILE_HPP */
