/*
 * Copyright 2018 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   CoordinateTransform.hpp
 * Author: glm,jordan
 *
 * Created on September 13, 2018, 3:39 PM
 */

#ifndef COORDINATETRANSFORM_HPP
#define COORDINATETRANSFORM_HPP

#include "../Position.hpp"
#include <Eigen/Dense>

class CoordinateTransform {
public:

    // WGS84 ellipsoid Parameters
    static constexpr double a = 6378137.0;
    static constexpr double e2 = 0.081819190842622 * 0.081819190842622;
    static constexpr double f = 1.0 / 298.257223563;
    static constexpr double b = a * one_f; // semi-minor axis
    static constexpr double epsilon = e2 / (1.0 - e2); // second eccentricity squared

    static void getPositionInNavigationFrame(Eigen::Vector3d & positionInNavigationFrame, const Position & positionGeographic, Eigen::Matrix3d & navDCM, Eigen::Vector3d & originECEF) {
        Eigen::Vector3d positionECEF;
        getPositionInTerrestialReferenceFrame(positionECEF, positionGeographic);

        Eigen::Vector3d positionVector = navDCM * (positionECEF - originECEF);

        positionInNavigationFrame << positionVector(0), positionVector(1), positionVector(2);
    };

    static void getPositionInTerrestialReferenceFrame(Eigen::Vector3d & positionECEF, const Position & position) {
        double N = a / (sqrt(1 - e2 * position.slat * position.slat));
        double xTRF = (N + position.ellipsoidalHeight) * position.clat * position.clon;
        double yTRF = (N + position.ellipsoidalHeight) * position.clat * position.slon;
        double zTRF = (N * (1 - e2) + position.ellipsoidalHeight) * position.slat;

        positionECEF << xTRF, yTRF, zTRF;
    };

    static void convertTerrestialReferenceFrameToLongitudeLatitudeElevation(Eigen::Vector3d & positionInNavigationFrame, Position & positionGeographic) {
        double xTRF = positionInNavigationFrame(0);
        double yTRF = positionInNavigationFrame(1);
        double zTRF = positionInNavigationFrame(2);
        
        // TODO: implement Bowring (1985) algorithm
    }

    static void getTerrestialToLocalGeodeticReferenceFrameMatrix(Eigen::Matrix3d & trf2lgf, Position & position) {
        double m00 = -position.slat * position.clon;
        double m01 = -position.slat * position.slon;
        double m02 = position.clat;

        double m10 = -position.slon;
        double m11 = position.clon;
        double m12 = 0;

        double m20 = -position.clat * position.clon;
        double m21 = -position.clat * position.slon;
        double m22 = -position.slat;

        trf2lgf <<
                m00, m01, m02,
                m10, m11, m12,
                m20, m21, m22;
    };
};

#endif /* COORDINATETRANSFORM_HPP */

