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
    
    // gradient inferior to this epsilon is considered 0
    // TODO: find a physically significant value for epsilon
    static constexpr double gradientEpsilon = 0.000001; 
    
    static void constantCelerityRayTracing(double z0, double z1, double c, double snellConstant, double & deltaZ, double & deltaR, double & deltaTravelTime) {
        double cosBn   = snellConstant*c;
        double sinBn   = sqrt(1 - pow(cosBn, 2));
        
        deltaZ = z1 - z0;
        deltaTravelTime = deltaZ/(c*sinBn);
        deltaR = cosBn*deltaTravelTime*c;
    }
    
    static void constantGradientRayTracing(double c0, double c1, double gradient, double snellConstant, double & deltaZ, double & deltaR, double & deltaTravelTime) {
        double cosBnm1 = snellConstant*c0;
        double cosBn   = snellConstant*c1;
        double sinBnm1 = sqrt(1 - pow(cosBnm1, 2));
        double sinBn   = sqrt(1 - pow(cosBn, 2));
        
        double radiusOfCurvature = 1.0/(snellConstant*gradient);
        
        deltaTravelTime = abs((1./abs(gradient))*log((c1/c0)*((1.0 + sinBnm1)/(1.0 + sinBn))));
        deltaZ = radiusOfCurvature*(cosBn - cosBnm1);
        deltaR = radiusOfCurvature*(sinBnm1 - sinBn);
    }
    
    static double soundSpeedGradient(double z0, double c0, double z1, double c1) {
        if(z1 == z0) {
            //this happens when svp contains multiple entries at same depth
            std::stringstream ss;
            ss << "Can't calculate gradient for svp samples at same depth: z0=" << z0 << " z1=" << z1;
            throw std::invalid_argument(ss.str());
        }
        
        return (c1 - c0) / (z1 - z0);
    }
    
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
        
        double currentLayerRaytraceTime = 0;
        double currentLayerDeltaZ = 0;
        double currentLayerDeltaR = 0;
        
        double cumulativeRaytraceTime = 0;
        double cumulativeRayX = 0;
        double cumulativeRayZ = 0;
        
        double oneWayTravelTime = (ping.getTwoWayTravelTime()/(double)2);
        
        Eigen::VectorXd depths = svp.getDepths();
        Eigen::VectorXd speeds = svp.getSpeeds();
        Eigen::VectorXd gradient = svp.getSoundSpeedGradient();

        //Ray tracing in first layer using sound speed at transducer
        //Snell's law's coefficient, using sound speed at transducer
        double snellConstant = cos(beta0)/ping.getSurfaceSoundSpeed();
        unsigned int svpCutoffIndex = svp.getLayerIndexForDepth(ping.getTransducerDepth()); //test this
        double gradientTransducerSvp = soundSpeedGradient(ping.getTransducerDepth(), ping.getSurfaceSoundSpeed(), depths(svpCutoffIndex), speeds(svpCutoffIndex));
        
        
        if(abs(gradientTransducerSvp) < gradientEpsilon) {
            constantCelerityRayTracing(
                ping.getTransducerDepth(),
                depths(svpCutoffIndex),
                ping.getSurfaceSoundSpeed(),
                snellConstant,
                currentLayerDeltaZ,
                currentLayerDeltaR,
                currentLayerRaytraceTime
            );
        } else {
            constantGradientRayTracing(
                ping.getSurfaceSoundSpeed(),
                speeds(svpCutoffIndex),
                gradientTransducerSvp,
                snellConstant,
                currentLayerDeltaZ,
                currentLayerDeltaR,
                currentLayerRaytraceTime
            );
        }
        
        //To ensure to work with the currentLayerIndex-1 cumulated travel time
        if (cumulativeRaytraceTime + currentLayerRaytraceTime <= oneWayTravelTime)
        {
            cumulativeRayX += currentLayerDeltaR;
            cumulativeRayZ += currentLayerDeltaZ;
            cumulativeRaytraceTime += currentLayerRaytraceTime;
        }
        
        unsigned int currentLayerIndex = svpCutoffIndex;
        while( (cumulativeRaytraceTime + currentLayerRaytraceTime)<=oneWayTravelTime //ray tracing time must be smaller than oneWayTravelTime
                && (currentLayerIndex<svp.getSize()-1) // stop before last layer
        ) {
            if (abs(gradient(currentLayerIndex)) < gradientEpsilon)
            {
                constantCelerityRayTracing(
                    depths(currentLayerIndex),
                    depths(currentLayerIndex+1),
                    speeds(currentLayerIndex),
                    snellConstant,
                    currentLayerDeltaZ,
                    currentLayerDeltaR,
                    currentLayerRaytraceTime
                );
            }
            else {
                constantGradientRayTracing(
                    speeds(currentLayerIndex),
                    speeds(currentLayerIndex+1),
                    gradient(currentLayerIndex),
                    snellConstant,
                    currentLayerDeltaZ,
                    currentLayerDeltaR,
                    currentLayerRaytraceTime
                );
            }

            //To ensure to work with the currentLayerIndex-1 cumulated travel time
            if (cumulativeRaytraceTime + currentLayerRaytraceTime <=  oneWayTravelTime)
            {
                currentLayerIndex++;
                cumulativeRayX += currentLayerDeltaR;
                cumulativeRayZ += currentLayerDeltaZ;
                cumulativeRaytraceTime += currentLayerRaytraceTime;
            } else {
                break; // this layer's travel time causes overshoot for onewaytraveltime
            }
        }
       
        // Last Layer Propagation
        double c_lastLayer = speeds(currentLayerIndex);
        double cosBn   = snellConstant*c_lastLayer;
        double sinBn   = sqrt(1 - pow(cosBn, 2));
        double lastLayerTraveTime = oneWayTravelTime - cumulativeRaytraceTime;
        double dxf = c_lastLayer*lastLayerTraveTime*cosBn;
        double dzf = c_lastLayer*lastLayerTraveTime*sinBn;

        // Output variable computation
        double Xf = cumulativeRayX + dxf;
        double Zf = cumulativeRayZ + dzf;

        raytracedPing(0) = Xf*sinAz;
        raytracedPing(1) = Xf*cosAz;
        raytracedPing(2) = Zf;
    }
};

#endif
