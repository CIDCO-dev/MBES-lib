/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   Ping.hpp
 * Author: glm,jordan
 *
 * Created on September 14, 2018, 10:10 AM
 */

#ifndef PING_HPP
#define PING_HPP

#include <iostream>
#include <Eigen/Dense>
#include "Attitude.hpp"
#include "Position.hpp"

class Ping {
private:
    uint64_t timestamp; //in microseconds since epoch
    uint64_t id;
    uint32_t quality;
    uint32_t intensity;

    double surfaceSoundSpeed;
    double twoWayTravelTime;
    double alongTrackAngle;  // In degrees, AKA emission angle, alpha, kappa or tilt angle
    double acrossTrackAngle; // In degrees, AKA reception angle, beta, zeta, beam angle


    /*Trigonometry is stored to prevent redundant recalculations*/
    double sA;
    double cA;

    double sB;
    double cB;

public:
    Ping(
	uint64_t microEpoch,
	long     id,
	uint32_t quality,
	uint32_t intensity,

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

    ~Ping() {

    }

    double getAcrossTrackAngle() {
        return acrossTrackAngle;
    }

    double getAlongTrackAngle() {
        return alongTrackAngle;
    }

    double getCA() {
        return cA;
    }

    double getCB() {
        return cB;
    }

    double getTimestamp() {
        return timestamp;
    }

    double getSA(){
	return sA;
    }

    double getSB() {
        return sB;
    }

    double getSurfaceSoundSpeed(){
	return surfaceSoundSpeed;
    }

    double getTwoWayTravelTime() {
        return twoWayTravelTime;
    }

};


#endif /* PING_HPP */

