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
    // WGS84 position
    const double latitude;
    const double longitude;
    const double ellipsoidalHeight;

    /*Trigonometry is stored to prevent redundant recalculations*/
    const double slat;
    const double clat;
    const double slon;
    const double clon;

    Position(double latitude, double longitude, double ellipsoidalHeight) :
    latitude(latitude),
    longitude(longitude),
    ellipsoidalHeight(ellipsoidalHeight),
    slat(sin(latitude * D2R)),
    clat(cos(latitude * D2R)),
    slon(sin(longitude * D2R)),
    clon(cos(longitude * D2R)) {
    };

    ~Position() {
    };

    friend std::ostream& operator<<(std::ostream& os, const Position& obj) {
        return os << "Latitude: " << obj.latitude << std::endl << "Longitude: " << obj.longitude << std::endl << "Ellipsoidal Height: " << obj.ellipsoidalHeight << std::endl;
    };
};

#endif /* POSITION_HPP */

