/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef RAYTRACING_HPP
#define RAYTRACING_HPP

#include <vector>
#include "../svp/SoundVelocityProfile.hpp"
#include "../Ping.hpp"
#include "../math/CoordinateTransform.hpp"


/*!
 * \brief Raytracing class
 * \author Guillaume Labbe-Morissette, Rabine Keyetieu
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
    static void rayTrace(Eigen::Vector3d & raytracedPing,Ping & ping,SoundVelocityProfile & svp, Eigen::Matrix3d & boresightMatrix,Eigen::Matrix3d & imu2nav){

	/*
	 * Compute launch vector
         */

	Eigen::Vector3d launchVectorSonar; //in sonar frame
	CoordinateTransform::sonar2cartesian(launchVectorSonar,ping.getAlongTrackAngle(),ping.getAcrossTrackAngle(), 1.0 ); 
        
#ifdef DEBUG
        std::cerr << "Launch vector: " << std::endl << launchVectorSonar << std::endl << std::endl;
#endif
        
	launchVectorSonar.normalize();

#ifdef DEBUG
        std::cerr << "Unit launch vector: " << std::endl << launchVectorSonar << std::endl << std::endl;
#endif        
        
	//convert to navigation frame where the raytracing occurs
	Eigen::Vector3d launchVectorNav = imu2nav * (boresightMatrix * launchVectorSonar);
        
#ifdef DEBUG
        std::cerr << "Launch vector in nav frame: " << std::endl << launchVectorNav << std::endl << std::endl;
#endif                

        double vNorm = sqrt(pow(launchVectorNav(0), 2)  + pow(launchVectorNav(1), 2));
        
	double sinAz= (vNorm >0)?launchVectorNav(0)/ vNorm : 0;
	double cosAz= (vNorm >0)?launchVectorNav(1)/ vNorm : 0;
	double beta0 = asin(launchVectorNav(2));
        
#ifdef DEBUG
        std::cerr << "sinAZ: " << sinAz << std::endl;
        std::cerr << "cosAz: " << cosAz << std::endl;
        std::cerr << "beta0: " << beta0 << std::endl << std::endl;
#endif        

        //compute gradient for each layer
        std::vector<double> gradient;
        for (unsigned int k=0; k < svp.getSize()-1; k++){
                gradient.push_back( (svp.getSpeeds()[k+1]- svp.getSpeeds()[k])/(svp.getDepths()[k+1]- svp.getDepths()[k]) );
        }
        
        //Snell's law's coefficient, using the first layer
        double epsilon = cos(beta0)/svp.getSpeeds()[0];
        
       unsigned int N = 0;
       
       double sinBn     = 0;
       double sinBnm1   = 0;
       double cosBn     = 0;
       double cosBnm1   = 0;
       double DT        = 0;
       double dtt       = 0;
       double DZ        = 0;
       double DR        = 0;
       double radiusOfCurvature = 0;
       double xff       = 0;
       double zff       = 0;
       
        while((DT + dtt)<= (ping.getTwoWayTravelTime()/(double)2) && (N<svp.getSize()-1)){
                //update angles
                sinBnm1 = sqrt(1 - pow(epsilon*svp.getSpeeds()[N], 2));
                sinBn   = sqrt(1 - pow(epsilon*svp.getSpeeds()[N+1], 2));

                cosBnm1 = epsilon*svp.getSpeeds()[N];
                cosBn   = epsilon*svp.getSpeeds()[N+1];

                if (abs(gradient[N]) < 0.000001) //FIXME: use a global epsilon value?
		{
                        //celerity gradient is zero so constant celerity in this layer
                        //delta t, delta z and r for the layer N
                        DZ = svp.getDepths()[N+1] - svp.getDepths()[N];
                        dtt = DZ/(svp.getSpeeds()[N]*sinBn);
                        DR = cosBn*dtt*svp.getSpeeds()[N];
		}
                else {
                        // if not null gradient
                        //Radius of curvature
                        radiusOfCurvature = 1.0/(epsilon*gradient[N]);

                        //delta t, delta z and r for the layer N
                        dtt = abs( (1./abs(gradient[N]))*log( (svp.getSpeeds()[N+1]/svp.getSpeeds()[N])*( (1.0 + sinBnm1)/(1.0 + sinBn) ) ) );
                        DZ = radiusOfCurvature*(cosBn - cosBnm1);
                        DR = radiusOfCurvature*(sinBnm1 - sinBn);
                }

                //To ensure to work with the N-1 cumulated travel time
                if (DT + dtt <=  (ping.getTwoWayTravelTime()/(double)2))
                {
                        N = N+1;
                        xff = xff + DR;
                        zff = zff + DZ;
                        DT = DT + dtt;
                }

        }

        // Last Layer Propagation
        double dtf = (ping.getTwoWayTravelTime()/(double)2) - DT;
        double dxf = svp.getSpeeds()[N]*dtf*cosBn;
        double dzf = svp.getSpeeds()[N]*dtf*sinBn;

        // Output variable computation
        double Xf = xff + dxf;
        double Zf = zff + dzf;

        raytracedPing(0) = Xf*sinAz;
        raytracedPing(1) = Xf*cosAz;
        raytracedPing(2) = Zf;
    }
};

#endif
