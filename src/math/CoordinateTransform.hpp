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
    static constexpr double b = a * (1-f); // semi-minor axis
    static constexpr double epsilon = e2 / (1.0 - e2); // second eccentricity squared

    static void getPositionInNavigationFrame(Eigen::Vector3d & positionInNavigationFrame, const Position & positionGeographic, Eigen::Matrix3d & navDCM, Eigen::Vector3d & originECEF) {
        Eigen::Vector3d positionECEF;
        getPositionInTerrestialReferenceFrame(positionECEF, positionGeographic);

        Eigen::Vector3d positionVector = navDCM * (positionECEF - originECEF);

        positionInNavigationFrame << positionVector(0), positionVector(1), positionVector(2);
    };

    static void getPositionECEF(Eigen::Vector3d & positionECEF, const Position & position) {
        double N = a / (sqrt(1 - e2 * position.getSlat() * position.getSlat()));
        double xTRF = (N + position.ellipsoidalHeight) * position.getClat() * position.getClon();
        double yTRF = (N + position.ellipsoidalHeight) * position.getClat() * position.getSlon();
        double zTRF = (N * (1 - e2) + position.ellipsoidalHeight) * position.getSlat();



        positionECEF << xTRF, yTRF, zTRF;
    };

    static void convertECEFToLongitudeLatitudeElevation(Eigen::Vector3d & positionInNavigationFrame, Position & positionGeographic) {
        double x = positionInNavigationFrame(0);
        double y = positionInNavigationFrame(1);
        double z = positionInNavigationFrame(2);

        // Bowring (1985) algorithm
        p2 = x * x + y*y;
        r2 = p2 + z*z;
        p = std::sqrt(p2);
        r = std::sqrt(r2);

        double tanu = (1 - f) * (z / p) * (1 + epsilon * b / r);
        double tan2u = tanu * tanu;

        double cos2u = 1.0 / (1.0 + tan2u);
        double cosu = std::sqrt(cos2u);
        double cos3u = cos2u * cosu;

        double sinu = tanu * cosu;
        double sin2u = 1.0 - cos2u;
        double sin3u = sin2u * sinu;
        
        double tanlat = (z + epsilon * b * sin3u) / (p - e2 * a * cos3u);
        
        double tan2lat = tanlat * tanlat;
        double cos2lat = 1.0 / (1.0 + tan2lat); 
        double sin2lat = 1.0 - cos2lat;
        
        double coslat = std::sqrt(cos2lat); 
        double sinlat = tanlat * coslat;
        
        double longitude = std::atan2(y, x);
        double latitude = std::atan(tanlat);
        double height = p * coslat + z * sinlat - a * sqrt(1.0 - e2 * sin2lat);
        
        positionGeographic.setLatitude(latitude);
        positionGeographic.setLongitude(longitude);
        positionGeographic.setEllipsoidalHeight(height);
    }

    static void getTerrestialToLocalGeodeticReferenceFrameMatrix(Eigen::Matrix3d & trf2lgf, Position & position) {
        double m00 = -position.getSlat() * position.getClon();
        double m01 = -position.getSlat() * position.getSlon();
        double m02 = position.getClat();

        double m10 = -position.getSlon();
        double m11 = position.getClon();
        double m12 = 0;

        double m20 = -position.getClat() * position.getClon();
        double m21 = -position.getClat() * position.getSlon();
        double m22 = -position.getSlat();

        trf2lgf <<
                m00, m01, m02,
                m10, m11, m12,
                m20, m21, m22;
    };
};

#endif /* COORDINATETRANSFORM_HPP */

