/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   Position.hpp
 * Author: glm,jordan
 *
 * Created on September 13, 2018, 3:29 PM
 */

#ifndef POSITION_HPP
#define POSITION_HPP

#include <iostream>
#include "utils/Constants.hpp"
#include <cmath>

class Position {
public:

    Position(double latitude, double longitude, double ellipsoidalHeight) :
    	latitude(latitude),
    	longitude(longitude),
    	ellipsoidalHeight(ellipsoidalHeight),
    	slat(sin(latitude * D2R)),
    	clat(cos(latitude * D2R)),
    	slon(sin(longitude * D2R)),
    	clon(cos(longitude * D2R)) 
     {};

    ~Position() {
    };

    uint64_t getMicroEpoch()		{ return microEpoch; };
    void     setMicroEpoch(uint64_t e)	{ microEpoch = e;};

    double   getLatitude()		{ return latitude; };
    void     setLatitude(double l)	{ latitude = l; slat=sin(latitude * D2R); clat=cos(latitude * D2R)};

    double   getLongitude()		{ return longitude; };
    void     setLongitude(double l)     { longitude = l; slon=sin(longitude * D2R);clon(cos(longitude * D2R)};

    double   getEllipsoidalHeight()     	{ return ellipsoidalHeight; };
    void     setEllipsoidalHeight(double h) 	{ ellipsoidalHeight = h;};
    
    double   getSlat()		{ return slat; };
    double   getSlon()		{ return slon; };
    double   getClat()		{ return clat; };
    double   getClon()		{ return clon; };

private:
    uint64_t microEpoch;

    // WGS84 position
    double latitude;
    double longitude;
    double ellipsoidalHeight;

    /*Trigonometry is stored to prevent redundant recalculations*/
    double slat;
    double clat;
    double slon;
    double clon;

    friend std::ostream& operator<<(std::ostream& os, const Position& obj) {
        return os << "Latitude: " << obj.latitude << std::endl << "Longitude: " << obj.longitude << std::endl << "Ellipsoidal Height: " << obj.ellipsoidalHeight << std::endl;
    };
};

#endif /* POSITION_HPP */

