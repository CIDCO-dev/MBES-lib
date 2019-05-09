/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   Ping.hpp
 * Author: glm,jordan, emilegagne
 *
 * Created on September 14, 2018, 10:10 AM
 */

#ifndef PING_HPP
#define PING_HPP

#include <iostream>
#include <Eigen/Dense>
#include "Attitude.hpp"
#include "Position.hpp"

/*!
 * \brief Ping class
 */
class Ping {
private:
    
    /**Time value calculated since January 1970 (micro-second)*/
    uint64_t timestamp; //in microseconds since epoch
    
    /**Value of the identification of the ping*/
    uint64_t id;
    
    /**Value of the quality of the ping*/
    uint32_t quality;
    
    /**Value of the intensity of the ping*/
    int32_t intensity;

    /**The sound speed value of the surface*/
    double surfaceSoundSpeed;
    
    /**Time value of transition between two points (micro-second)*/
    double twoWayTravelTime;
    
    /**Value of the angle who pass along the track (degrees)*/
    double alongTrackAngle;  // In degrees, AKA emission angle, alpha, kappa or tilt angle
    
    /**Value of the angle who pass across the track (degrees)*/
    double acrossTrackAngle; // In degrees, AKA reception angle, beta, zeta, beam angle


    /*Trigonometry is stored to prevent redundant recalculations*/
    /**Sine value of the along track angle*/
    double sA;
    
    /**Cosine value of the along track angle*/
    double cA;

    /**Sine value of the across track angle*/
    double sB;
    
    /**Cosine value of the across track angle*/
    double cB;


public:
    
    /**
     * Create the ping
     * 
     * @param microEpoch timestamp value of the ping
     * @param id identification of the ping
     * @param quality quality of the ping
     * @param intensity intensity of the ping
     * @param surfaceSoundSpeed the sound speed of the surface
     * @param twoWayTravelTime time value of the transition between
     * @param alongTrackAngle Angle who pass along the track
     * @param acrossTrackAngle Angle who pass across the track
     */
    Ping(
	uint64_t microEpoch,
	long     id,
	uint32_t quality,
	int32_t intensity,

        double surfaceSoundSpeed,
        double twoWayTravelTime,
        double alongTrackAngle,
        double acrossTrackAngle
    ):
    timestamp(microEpoch),
    id(id),
    quality(quality),
    intensity(intensity),
    surfaceSoundSpeed(surfaceSoundSpeed),
    twoWayTravelTime(twoWayTravelTime),
    alongTrackAngle(alongTrackAngle),
    acrossTrackAngle(acrossTrackAngle),
    sA(sin(alongTrackAngle*D2R)),
    cA(cos(alongTrackAngle*D2R)),
    sB(sin(acrossTrackAngle*D2R)),
    cB(cos(acrossTrackAngle*D2R)){
    }

    /** Destroy the ping*/
    ~Ping() {

    }

    /**Return the across track angle*/
    double getAcrossTrackAngle() {
        return acrossTrackAngle;
    }

    /**Return the along track angle*/
    double getAlongTrackAngle() {
        return alongTrackAngle;
    }

    /**Return the cosine value of the along track angle*/
    double getCA() {
        return cA;
    }

    /**Return the cosine value of the across track angle*/
    double getCB() {
        return cB;
    }

    /**Return the timestamp of the ping*/
    double getTimestamp() {
        return timestamp;
    }

    /**Return the sine value of the along track angle*/
    double getSA(){
	return sA;
    }

    /**Return the sine value of the across track angle*/
    double getSB() {
        return sB;
    }

    /**Return the sound speed of the surface*/
    double getSurfaceSoundSpeed(){
	return surfaceSoundSpeed;
    }

    /**Return the time value of the transition between two points*/
    double getTwoWayTravelTime() {
        return twoWayTravelTime;
    }

    /**Return the quality of the ping*/
    uint32_t getQuality() { return quality;}

    /**Return the intensity of the ping*/
    uint32_t getIntensity() { return intensity;}


    static bool sortByTimestamp(Ping & p1,Ping & p2){
        return p1.getTimestamp() < p2.getTimestamp(); 
    }


};


#endif /* PING_HPP */

