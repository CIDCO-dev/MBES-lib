/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   CoordinateTransform.hpp
 * Author: jordan
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

    static Eigen::Vector3d* getPositionInNavigationFrame(const Position & position, Eigen::Matrix3d & navDCM, Eigen::Vector3d & originTRF) {

        Eigen::Vector3d* positionTRF = getPositionInTerrestialReferenceFrame(position);
        Eigen::Vector3d positionVector = navDCM * (*positionTRF - originTRF);
        delete positionTRF;

        Eigen::Vector3d* positionInNavigationFrame = new Eigen::Vector3d();
        *positionInNavigationFrame << positionVector(0), positionVector(1), positionVector(2);

        return positionInNavigationFrame;
    };

    static Eigen::Vector3d * getPositionInTerrestialReferenceFrame(const Position & position) {
        double N = a / (sqrt(1 - e2 * position.slat * position.slat));
        double xTRF = (N + position.ellipsoidalHeight) * position.clat * position.clon;
        double yTRF = (N + position.ellipsoidalHeight) * position.clat * position.slon;
        double zTRF = (N * (1 - e2) + position.ellipsoidalHeight) * position.slat;

        Eigen::Vector3d * positionInTerrestialReferenceFrame = new Eigen::Vector3d();
        *positionInTerrestialReferenceFrame << xTRF, yTRF, zTRF;

        return positionInTerrestialReferenceFrame;
    };

    static Eigen::Matrix3d * getTerrestialToLocalGeodeticReferenceFrameMatrix(Position & position) {
        double m00 = -position.slat*position.clon;
        double m01 = -position.slat*position.slon;
        double m02 = position.clat;

        double m10 = -position.slon;
        double m11 = position.clon;
        double m12 = 0;

        double m20 = -position.clat*position.clon;
        double m21 = -position.clat*position.slon;
        double m22 = -position.slat;

        Eigen::Matrix3d * terrestialToLocalGeodeticReferenceFrameMatrix = new Eigen::Matrix3d();
        *terrestialToLocalGeodeticReferenceFrameMatrix <<
                m00, m01, m02,
                m10, m11, m12,
                m20, m21, m22;

        return terrestialToLocalGeodeticReferenceFrameMatrix;
    };
};

#endif /* COORDINATETRANSFORM_HPP */

