/*
 * Copyright 2017-2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   Georeferencing.hpp
 * Author: glm,jordan, emilegagne
 *
 * Created on October 2, 2018, 9:39 AM
 */

#ifndef GEOREFERENCING_HPP
#define GEOREFERENCING_HPP

#include <Eigen/Dense>
#include "math/CoordinateTransform.hpp"
#include "Raytracing.hpp"
/*!
 * \brief Georeferencing class
 */
class Georeferencing{
public:

    /**
     * Create a georeferencing
     * 
     * @param georeferencedPing vector of a ping georeferenced
     * @param attitude the attitude of the ship in the IM frame
     * @param position the position of the ship in the TRF
     * @param ping the ping of the georeference in the sonar frame
     * @param svp the sound velocity profile
     * @param leverArm vector from the position reference point (PRP) to the acoustic center
     * 
     */
    static void georeference(Eigen::Vector3d & georeferencedPing,Attitude & attitude,Position & position,Ping & ping,SoundVelocityProfile & svp,Eigen::Vector3d & leverArm,Eigen::Matrix3d & boresight) {
	//Compute transform matrixes
        Eigen::Matrix3d ned2ecef;
        CoordinateTransform::ned2ecef(ned2ecef,position);

        Eigen::Matrix3d imu2ned;
        CoordinateTransform::getDCM(imu2ned,attitude);

	//Convert position to ECEF
	Eigen::Vector3d positionECEF;
	CoordinateTransform::getPositionECEF(positionECEF,position);

	//Convert ping to ECEF
	Eigen::Vector3d pingVector;
	Raytracing::rayTrace(pingVector,ping,svp);

	Eigen::Vector3d pingECEF = ned2ecef * (imu2ned * boresight * pingVector);

	//Convert lever arm to ECEF
	Eigen::Vector3d leverArmECEF =  ned2ecef * (imu2ned * leverArm);

	//Compute total ECEF vector

	georeferencedPing = positionECEF + pingECEF + leverArmECEF;
    }
};



#endif
