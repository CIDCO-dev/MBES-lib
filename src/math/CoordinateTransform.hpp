/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef COORDINATETRANSFORM_HPP
#define COORDINATETRANSFORM_HPP

#include "../Position.hpp"
#include "../Attitude.hpp"
#include "../utils/Constants.hpp"
#include <Eigen/Dense>


/*!
* \brief Coordinate transform class
* \author Guillaume Labbe-Morissette, Jordan McManus
* \date September 13, 2018, 3:39 PM
*/
class CoordinateTransform {
public:

  // WGS84 ellipsoid Parameters

  /**WGS84 ellipsoid semi-major axis*/
  static constexpr double a = 6378137.0;

  /**WGS84 ellipsoid first eccentricity squared*/
  static constexpr double e2 = 0.081819190842622 * 0.081819190842622;

  /**WGS84 ellipsoid inverse flattening*/
  static constexpr double f = 1.0 / 298.257223563;

  /**WGS84 ellipsoid semi-minor axis*/
  static constexpr double b = a * (1-f); // semi-minor axis

  /**WGS84 ellipsoid second eccentricity squared*/
  static constexpr double epsilon = e2 / (1.0 - e2); // second eccentricity squared

  /**
  * Sets the position in navigation frame
  *
  * @param positionInNavigationFrame the value that needs to be set
  * @param positionGeographic the geographic position
  * @param navDCM the rotation matrix
  * @param originECEF the ECEF origin position
  */
  static void getPositionInNavigationFrame(Eigen::Vector3d & positionInNavigationFrame, Position & positionGeographic, Eigen::Matrix3d & navDCM, Eigen::Vector3d & originECEF) {
    Eigen::Vector3d positionECEF;
    getPositionECEF(positionECEF, positionGeographic);

    Eigen::Vector3d positionVector = navDCM * (positionECEF - originECEF);

    positionInNavigationFrame << positionVector(0), positionVector(1), positionVector(2);
  };

  /**
  * Sets the ECEF position by the given position
  *
  * @param positionECEF value that needs to be set
  * @param position the position used to get the ECEF position
  */
  static void getPositionECEF(Eigen::Vector3d & positionECEF, Position & position) {
    double N = a / (sqrt(1 - e2 * position.getSlat() * position.getSlat()));
    double xTRF = (N + position.getEllipsoidalHeight()) * position.getClat() * position.getClon();
    double yTRF = (N + position.getEllipsoidalHeight()) * position.getClat() * position.getSlon();
    double zTRF = (N * (1 - e2) + position.getEllipsoidalHeight()) * position.getSlat();

    positionECEF << xTRF, yTRF, zTRF;
  };

  /**
  * Gets the longitude, latitude and elevation of an ECEF position
  *
  * @param positionInNavigationFrame the position we need to get the latitude, longitude and elevation
  * @param positionGeographic the position where the latitude, longitude et elevation will be put in
  */
  static void convertECEFToLongitudeLatitudeElevation(Eigen::Vector3d & positionInNavigationFrame, Position & positionGeographic) {
    double x = positionInNavigationFrame(0);
    double y = positionInNavigationFrame(1);
    double z = positionInNavigationFrame(2);

    // Bowring (1985) algorithm
    double p2 = x * x + y*y;
    double r2 = p2 + z*z;
    double p = std::sqrt(p2);
    double r = std::sqrt(r2);

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

    positionGeographic.setLatitude(latitude*R2D);
    positionGeographic.setLongitude(longitude*R2D);
    positionGeographic.setEllipsoidalHeight(height);
  }

  /**
  * Sets a terrestrial to a local geodetic reference frame matrix by the given position
  *
  * @param trf2lgf the terrestrial to local geodetic reference frame matrix that needs to be set
  * @param position the given position
  */
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

  /**
  * Sets rotation matrix from neutral/zero position to the given attitude
  *
  * @param outputMatrix matrix that needs to be set
  * @param attitude the given attitude
  */
  static void getDCM(Eigen::Matrix3d & outputMatrix,Attitude & attitude){
    outputMatrix <<         attitude.getCh()*attitude.getCp(),   attitude.getCh()*attitude.getSp()*attitude.getSr()-attitude.getCr()*attitude.getSh(), attitude.getCh()*attitude.getCr()*attitude.getSp()+attitude.getSr()*attitude.getSh(),
    attitude.getCp()*attitude.getSh(),   attitude.getCh()*attitude.getCr()+attitude.getSp()*attitude.getSr()*attitude.getSh(), attitude.getSh()*attitude.getCr()*attitude.getSp()-attitude.getCh()*attitude.getSr(),
    -attitude.getSp(),          attitude.getCp()*attitude.getSr(),        attitude.getCr()*attitude.getCp();
  }


  /**
  * NED Tangent plane at position to WGS84 ECEF
  *
  * @param outputMatrix the matrix that needs to be set
  * @param position the position to convert
  */
  static void ned2ecef(Eigen::Matrix3d & outputMatrix,Position & position){

    outputMatrix << -position.getClon()*position.getSlat(),-position.getSlon(),-position.getClat()*position.getClon(),
    -position.getSlat()*position.getSlon(),position.getClon(),-position.getClat()*position.getSlon(),
    position.getClat(),0,-position.getSlat();

  }


  /**
  * Converts spherical coordinates to cartesian
  *
  * @param outputVector the cartesian coordinates
  * @param theta the polar angle
  * @param phi the azimuthal angle
  * @param r the radial distance
  */
  static void spherical2cartesian(Eigen::Vector3d & outputVector,double theta,double phi,double r){
    outputVector(0) = r * sin(D2R*theta)*cos(D2R*phi);
    outputVector(1) = r * sin(D2R*theta)*sin(D2R*phi);
    outputVector(2) = r * cos(D2R*theta);
  }

  /**
  * Converts sonar coordinates (alpha,beta,r) to cartesian (NED)
  *
  * @param outputVector the cartesian coordinates
  * @param aphaDegrees the alpha degree
  * @param betaDegrees the beta degree
  * @param r the radical distance
  */
  static void sonar2cartesian(Eigen::Vector3d & outputVector,double alphaDegrees,double betaDegrees,double r){
    outputVector(0)=r * sin(alphaDegrees*D2R);
    outputVector(1)=r * cos(alphaDegrees*D2R) * sin(betaDegrees*D2R);
    outputVector(2)=r * cos(alphaDegrees*D2R) * cos(betaDegrees*D2R);
  }
};

#endif /* COORDINATETRANSFORM_HPP */
