/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   Position.hpp
 * Author: glm,jordan,emilegagne
 *
 * Created on September 13, 2018, 3:29 PM
 */

#ifndef POSITION_HPP
#define POSITION_HPP

#include <iostream>
#include "utils/Constants.hpp"
#include <cmath>

/*!
 * \brief Position class
 */
class Position {
public:

    /**
     * Create a position
     * 
     * @param microEpoch time value calculated since January 1970 (micro-second)
     * @param latitude the latitude of the position
     * @param longitude the longitude of the position
     * @param ellipsoidalHeight the ellipsoidal height of the position
     */
    Position(uint64_t microEpoch,double latitude, double longitude, double ellipsoidalHeight) :
    	timestamp(microEpoch),
    	slat(sin(latitude * D2R)),
    	clat(cos(latitude * D2R)),
    	slon(sin(longitude * D2R)),
    	clon(cos(longitude * D2R)),
	vector(latitude,longitude,ellipsoidalHeight)
    {}

    /**Destroy the position*/
    ~Position() {
    }

    /**Return the timestamp of the position*/
    uint64_t getTimestamp()		{ return timestamp; }
    
    /**
     * Change the timestamp of the position
     * 
     * @param e the new timestamp value
     */
    void     setTimestamp(uint64_t e)	{ timestamp = e;}

    /**Return the latitude of the position*/
    double   getLatitude()		{ return vector(0); }
    
    /**
     * Change the latitude of the position
     * 
     * @param l the new latitude
     */
    void     setLatitude(double l)	{ vector(0)=l; slat=sin(vector(0) * D2R); clat=cos(vector(0) * D2R);}

    /**Return the longitude of the position*/
    double   getLongitude()		{ return vector(1); }
    
    /**
     * Change the longitude of the position
     * 
     * @param l the new longitude
     */
    void     setLongitude(double l)     { vector(1)=l; slon=sin(vector(1) * D2R);clon=cos(vector(1) * D2R);}

    /**Return the ellipsoidal heigh of the position*/
    double   getEllipsoidalHeight()     	{ return vector(2); }
    
    /**
     * Change the ellipsoidal heigh of the position
     * 
     * @param h the new ellipsoidal height
     */
    void     setEllipsoidalHeight(double h) 	{ vector(2)=h;}

    /**Return the sine value of the latitude*/
    double   getSlat()		{ return slat; }
    
    /**Return the sine value of the longitude*/
    double   getSlon()		{ return slon; }
    
    /**Return the cosine value of the latitude*/
    double   getClat()		{ return clat; }
    
    /**Return the cosine value of the longitude*/
    double   getClon()		{ return clon; }

    /**Return the vectorized form of the position*/
    Eigen::Vector3d & getVector() { return vector;}

    static bool sortByTimestamp(Position & p1,Position & p2){
    	return p1.getTimestamp() < p2.getTimestamp(); 
    }

private:
    
    /**Timestamp value of the position (micro-second)*/
    uint64_t timestamp;

    /*Trigonometry is stored to prevent redundant recalculations*/
    
    /**Sine value of the latitude*/
    double slat;
    
    /**Cosine value of the latitude*/
    double clat;
    
    /**Sine value of the longitude*/
    double slon;
    
    /**Cosine value of the longitude*/
    double clon;

    /**Position vector in WGS84*/
    Eigen::Vector3d vector;

    /**
     * Return a text value who contain the informations of the position
     * 
     * @param os the text value who most contain the informations of the position
     * @param obj the position that we need to get the informations
     */
    friend std::ostream& operator<<(std::ostream& os, const Position& obj) {
        return os << "Latitude: " << obj.vector(0) << std::endl << "Longitude: " << obj.vector(1) << std::endl << "Ellipsoidal Height: " << obj.vector(2) << std::endl;
    }
};

#endif /* POSITION_HPP */

