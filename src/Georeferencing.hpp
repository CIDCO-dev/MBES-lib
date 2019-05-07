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
    /**
     * Georeference a ping
     * 
     * @param georeferencedPing georeferenced ping in vector form
     * @param attitude the attitude of the ship in the IMU frame
     * @param position the position of the ship in the TRF
     * @param ping the ping of the georeference in the sonar frame
     * @param svp the sound velocity profile
     * @param leverArm vector from the position reference point (PRP) to the acoustic center
     * 
     */
    static virtual void georeference(Eigen::Vector3d & georeferencedPing,Attitude & attitude,Position & position,Ping & ping,SoundVelocityProfile & svp,Eigen::Vector3d & leverArm,Eigen::Matrix3d & boresight){};
};

/*!
 * \brief TRF Georeferencing class
 */
class GeoreferencingTRF{
public:

    /**
     * Georeferences a ping in the TRF
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


/*!
 * \brief TRF Georeferencing class
 */
class GeoreferencingLGF{
public:

    /**
     * Georeferences a ping in the LGF (NED)
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

        //Convert position to NED
        Eigen::Vector3d positionECEF;
        CoordinateTransform::getPositionECEF(positionECEF,position);
	Eigen::Vector3d positionNED = positionECEF * ned2ecef.transpose();

        //Convert ping to NED
        Eigen::Vector3d pingVector;
        Raytracing::rayTrace(pingVector,ping,svp);

        Eigen::Vector3d pingNED = imu2ned * boresight * pingVector;

        //Convert lever arm to NED
        Eigen::Vector3d leverArmNED =  imu2ned * leverArm;

        //Compute total NED vector

        georeferencedPing = positionNED + pingNED + leverArmNED;
    }
};

#endif
