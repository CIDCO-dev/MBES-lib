/*
 * Copyright 2017-2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   Raytracing.hpp
 */

#ifndef RAYTRACING_HPP
#define RAYTRACING_HPP

#include "svp/SoundVelocityProfile.hpp"

/*!
 * \brief Raytracing class
 * \author ?
 */
class Raytracing{
public:
    /**
     * Makes a raytracing
     *
     * @param raytracedPing the raytraced ping for the raytracing
     * @param ping the Ping for the raytracing
     * @param svp the SoundVelocityProfile for the raytracing
     */
    static void rayTrace(Eigen::Vector3d & raytracedPing,Ping & ping,SoundVelocityProfile & svp){
	//TODO: do actual raytracing. This is just for quick testing purposes
	CoordinateTransform::sonar2cartesian(raytracedPing,ping.getAlongTrackAngle(),ping.getAcrossTrackAngle(), (ping.getTwoWayTravelTime()/(double)2) * (double)1480 );
    }
};

#endif
