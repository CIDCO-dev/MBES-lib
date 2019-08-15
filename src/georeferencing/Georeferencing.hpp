/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef GEOREFERENCING_HPP
#define GEOREFERENCING_HPP

#include <Eigen/Dense>
#include "../math/CoordinateTransform.hpp"
#include "Raytracing.hpp"
#include "../Ping.hpp"

/*!
* \brief Georeferencing class
* \author Guillaume Labbe-Morissette, Jordan McManus, Emile Gagne
* \date October 2, 2018, 9:39 AM
*/
class Georeferencing{
public:
  /**
  * Georeferences a ping
  *
  * @param georeferencedPing georeferenced ping in vector form
  * @param attitude the attitude of the ship in the IMU frame
  * @param position the position of the ship in the TRF
  * @param ping the ping of the georeference in the sonar frame
  * @param svp the SoundVelocityProfile
  * @param leverArm vector from the position reference point (PRP) to the acoustic center
  *
  */
  virtual void georeference(Eigen::Vector3d & georeferencedPing,Attitude & attitude,Position & position,Ping & ping,SoundVelocityProfile & svp,Eigen::Vector3d & leverArm,Eigen::Matrix3d & boresight){};
};

/*!
* \brief TRF Georeferencing class
*
* Extends Georeferencing class
*/
class GeoreferencingTRF : public Georeferencing{
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
  void georeference(Eigen::Vector3d & georeferencedPing,Attitude & attitude,Position & position,Ping & ping,SoundVelocityProfile & svp,Eigen::Vector3d & leverArm,Eigen::Matrix3d & boresight) {
    //Compute transform matrixes
    Eigen::Matrix3d ned2ecef;
    CoordinateTransform::ned2ecef(ned2ecef,position);
    
#ifdef DEBUG
        std::cerr << "NED 2 ECEF: " << std::endl << ned2ecef << std::endl << std::endl;
#endif

    Eigen::Matrix3d imu2ned;
    CoordinateTransform::getDCM(imu2ned,attitude);

#ifdef DEBUG
        std::cerr << "IMU 2 NED: " << std::endl << imu2ned << std::endl << std::endl;
#endif
    
    
    //Convert position to ECEF
    Eigen::Vector3d positionECEF;
    CoordinateTransform::getPositionECEF(positionECEF,position);
    
#ifdef DEBUG
        std::cerr << "Position ECEF: " << std::endl << positionECEF << std::endl << std::endl;
#endif
    

    //Convert ping to ECEF
    Eigen::Vector3d pingVectorNED;
    Raytracing::rayTrace(pingVectorNED,ping,svp,boresight,imu2ned);
    
#ifdef DEBUG
        std::cerr << "Raytraced ping: " << std::endl << pingVectorNED << std::endl << std::endl;
#endif
    

    Eigen::Vector3d pingECEF = ned2ecef * pingVectorNED;

#ifdef DEBUG
        std::cerr << "Ping ECEF: " << std::endl << pingECEF << std::endl << std::endl;
#endif
    
    
    //Convert lever arm to ECEF
    Eigen::Vector3d leverArmECEF =  ned2ecef * (imu2ned * leverArm);

    //Compute total ECEF vector

    georeferencedPing = positionECEF + pingECEF + leverArmECEF;
  }
};


/*!
* \brief LGF Georeferencing class
*/
class GeoreferencingLGF : public Georeferencing{
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
    
    void georeference(Eigen::Vector3d & georeferencedPing,Attitude & attitude,Position & position,Ping & ping,SoundVelocityProfile & svp,Eigen::Vector3d & leverArm,Eigen::Matrix3d & boresight) {
        Eigen::Matrix3d imu2ned;
        CoordinateTransform::getDCM(imu2ned,attitude);

	//Convert position's geographic coordinates to ECEF, and then from ECEF to NED
        Eigen::Vector3d positionECEF;
        CoordinateTransform::getPositionECEF(positionECEF,position);
        
#ifdef DEBUG
        std::cerr << "Position ECEF: " << std::endl << positionECEF << std::endl << std::endl;
#endif

        Eigen::Vector3d centered = positionECEF-centroidECEF;    

	Eigen::Vector3d positionNED = ecef2ned * centered;

        //Convert ping to NED
        Eigen::Vector3d pingNED;
        Raytracing::rayTrace(pingNED,ping,svp,boresight,imu2ned);

        //Convert lever arm to NED
        Eigen::Vector3d leverArmNED =  imu2ned * leverArm;

        //Compute total NED vector
        georeferencedPing = positionNED + pingNED + leverArmNED;
    }

    /**
     * Sets centroid and inits ECEF 2 NED matrix
     */
    void setCentroid(Position & c){
	if(this->centroid) delete centroid;

	this->centroid=new Position(c.getTimestamp(), c.getLatitude(), c.getLongitude(), c.getEllipsoidalHeight());
        CoordinateTransform::getPositionECEF(centroidECEF,*this->centroid);
	CoordinateTransform::ned2ecef(ecef2ned,*this->centroid);
        ecef2ned.transposeInPlace();
    }

    /**
    *  Get a pointer to the centroid
    */

    Position * getCentroid(){ return centroid;};

private:
	Position * centroid = NULL; //in geographic coordinates
        Eigen::Vector3d centroidECEF;
	Eigen::Matrix3d ecef2ned;
};

#endif
