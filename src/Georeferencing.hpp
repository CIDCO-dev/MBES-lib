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
     * @param georeferencedPing vector who give the georeference of a ping
     * @param attitude the attitude of the georeference
     * @param position the position of the georeference
     * @param ping the ping of the georeference
     * @param svp the song velocity of the georeference
     * @param leverArm vector of the leverArm who goes from the GPS to the acoustic
     * 
     */
    static void georeference(Eigen::Vector3d & georeferencedPing,Attitude & attitude,Position & position,Ping & ping,SoundVelocityProfile & svp,Eigen::Vector3d & leverArm) {
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

	Eigen::Vector3d pingECEF = ned2ecef * pingVector;

	//Convert lever arm to ECEF
	Eigen::Vector3d leverArmECEF =  ned2ecef * (imu2ned * leverArm);

	//Compute total ECEF vector

/*
        std::cerr <<"NED-to-ECEF" << std::endl;
        std::cerr << ned2ecef <<std::endl;
        std::cerr << "-------------" << std::endl;


	std::cerr <<"Position" << std::endl;
	std::cerr << positionECEF<<std::endl;
	std::cerr << "-------------" << std::endl;

        std::cerr << "Ping" << std::endl;
        std::cerr << pingECEF << std::endl;
        std::cerr << "-------------" << std::endl;


        std::cerr << "LA" << std::endl;
        std::cerr << leverArmECEF << std::endl;
        std::cerr << "-------------" << std::endl;
*/
	georeferencedPing = positionECEF + pingECEF + leverArmECEF;
    }
};



#endif
