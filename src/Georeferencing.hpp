/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   Georeferencing.hpp
 * Author: glm,jordan
 *
 * Created on September 12, 2018, 3:27 PM
 */

#ifndef GEOREFERENCING_HPP
#define GEOREFERENCING_HPP

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include "math/DCM.hpp"
#include "Ping.hpp"
#include "SoundVelocityProfile.hpp"
#include "math/CoordinateTransform.hpp"
#include <cmath>

class Georeferencing {
public:

    static void georeference(Eigen::Vector3d & outputVector,Ping & ping, Eigen::Vector3d & leverArm, Eigen::Vector3d & rayInNavigationFrame, Eigen::Vector3d & origin, Eigen::Matrix3d & navDCM) {
        Eigen::Matrix3d imuToNavigationFrameAtTransmission;
        DCM::getDcm(imuToNavigationFrameAtTransmission,*(ping.transmissionAttitude));

        Eigen::Vector3d rayInNavFrame = rayInNavigationFrame + imuToNavigationFrameAtTransmission * leverArm;

        Eigen::Vector3d positionReferencePoint;
	CoordinateTransform::getPositionInNavigationFrame(positionReferencePoint,*(ping.transmissionPosition), navDCM, origin);

        Eigen::Vector3d rayFromAntenna = positionReferencePoint + rayInNavFrame;

        outputVector << rayFromAntenna(0), rayFromAntenna(1), rayFromAntenna(2);
    };

    static void calculateLaunchVector(Eigen::Vector3d & outputVector,Ping & ping, Eigen::Matrix3d & boresight) {
        Eigen::Vector3d launchVectorInSonarFrame;
        launchVectorInSonarFrame << ping.sA, ping.cA * ping.sB, ping.cA * ping.cB;

        Eigen::Matrix3d imuToNavigationFrame;
        DCM::getDcmRollAtReceptionPitchHeadingAtEmission(imuToNavigationFrame,*(ping.receptionAttitude), *(ping.transmissionAttitude));
        Eigen::Vector3d launchVectorInNavigationFrame = imuToNavigationFrame * (boresight * launchVectorInSonarFrame);

        outputVector << launchVectorInNavigationFrame(0), launchVectorInNavigationFrame(1), launchVectorInNavigationFrame(2);
    }

    static void constantCelerityRayTracing(Eigen::Vector3d & outputVector,Eigen::Vector3d & launchVector, double celerity, double travelTime) {
        double dz = (celerity * travelTime) * launchVector(2);
        double dr = (celerity * travelTime) * sqrt(1 - launchVector(2) * launchVector(2));

        double radialNorm = sqrt(launchVector(0) * launchVector(0) + launchVector(1) * launchVector(1));

        if (radialNorm > 0) {
            outputVector << dr * launchVector(0) / radialNorm, dr * launchVector(1) / radialNorm, dz;
        } else {
            outputVector << 0, 0, dz;
        }

    };

    static void gradientCelerityRayTracing(Eigen::Vector3d & outputVector,Eigen::Vector3d & launchVector, SoundVelocityProfile & svp, double travelTime) {

        Eigen::VectorXd depths = svp.getDepth();
        Eigen::VectorXd celerity = svp.getSoundSpeed();
        Eigen::VectorXd gradient = svp.getGradient();

        unsigned int numberOfLayers = svp.getSize() - 1; // layers between each SVP elements

        double cosBeta0 = sqrt(1 - launchVector(2) * launchVector(2)); // launchVector(2) is sin(beta0)
        double epsilon = cosBeta0 / celerity(0); // Snell Descartes

        Eigen::VectorXd cosBeta = epsilon * celerity;
        Eigen::VectorXd sinBeta = (Eigen::VectorXd::Ones(svp.getSize()) - cosBeta.cwiseProduct(cosBeta)).cwiseSqrt();

        unsigned int currentLayer = 0;
        double cumulatedTracingTravelTime = 0.0;
        double cumulatedDepth = 0.0;
        double cumulatedRadialDistance = 0.0;
        while (/*cumulatedTracingTravelTime < travelTime &&*/ currentLayer < numberOfLayers) {

            double deltaTime;
            double deltaDepth;
            double deltaRadialDistance;

            if (gradient(currentLayer) != 0.0) {

                double radiusOfCurvature = 1. / (epsilon * gradient(currentLayer));
                deltaTime = abs((1. / abs(gradient(currentLayer))) * log((celerity(currentLayer + 1) / celerity(currentLayer))*((1.0 + sinBeta(currentLayer)) / (1.0 + sinBeta(currentLayer + 1)))));
                deltaDepth = radiusOfCurvature * (cosBeta(currentLayer + 1) - cosBeta(currentLayer));
                deltaRadialDistance = radiusOfCurvature * (sinBeta(currentLayer) - sinBeta(currentLayer + 1));
            } else {
                // constant celerity
                deltaDepth = depths(currentLayer + 1) - depths(currentLayer);
                deltaTime = deltaDepth / (celerity(currentLayer) * sinBeta(currentLayer + 1));
                deltaRadialDistance = cosBeta(currentLayer + 1) * deltaTime * celerity(currentLayer);
            }

            std::cout << "new raytracing step " << currentLayer << std::endl;
            std::cout << "cosBeta(currentLayer):" << cosBeta(currentLayer) << std::endl;
            std::cout << "deltaTime: " << deltaTime << std::endl;
            std::cout << "deltaRadialDistance: " << deltaRadialDistance << std::endl;
            std::cout << "deltaDepth: " << deltaDepth << std::endl;
            std::cin.get();

            if (cumulatedTracingTravelTime + deltaTime <= travelTime) {
                cumulatedDepth += deltaDepth;
                cumulatedRadialDistance += deltaRadialDistance;
                cumulatedTracingTravelTime += deltaTime;
            } else {
                break;
            }

            currentLayer++;
        }

        //TODO: last layer propagation
        double finalLayerTravelTime = travelTime - cumulatedTracingTravelTime;
        double deltaRadialDistanceFinalLayer = celerity(celerity.size() - 1) * finalLayerTravelTime * cosBeta(cosBeta.size() - 1);
        double deltaDepthFinalLayer = celerity(celerity.size() - 1) * finalLayerTravelTime * sinBeta(sinBeta.size() - 1);

        std::cout << "new raytracing Last Layer:" << std::endl;
        std::cout << "finalLayerTravelTime: " << finalLayerTravelTime << std::endl;
        std::cout << "deltaRadialDistanceFinalLayer: " << deltaRadialDistanceFinalLayer << std::endl;
        std::cout << "deltaDepthFinalLayer: " << deltaDepthFinalLayer << std::endl;
        std::cin.get();


	//project to NED
        cumulatedRadialDistance += deltaRadialDistanceFinalLayer;
        cumulatedDepth += deltaDepthFinalLayer;

        double radialNorm = sqrt(launchVector(0) * launchVector(0) + launchVector(1) * launchVector(1));

        outputVector << cumulatedRadialDistance * launchVector(0) / radialNorm, cumulatedRadialDistance * launchVector(1) / radialNorm, cumulatedDepth;
    }


};

#endif /* GEOREFERENCING_HPP */

