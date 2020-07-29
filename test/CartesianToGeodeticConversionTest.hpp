/*
 * Copyright 2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   CartesianToGeodeticConversionTest.hpp
 * Author: jordan
 */

#ifndef CARTESIANTOGEODETICCONVERSIONTEST_HPP
#define CARTESIANTOGEODETICCONVERSIONTEST_HPP

#include "catch.hpp"
#include <Eigen/Dense>
#include "../src/Position.hpp"
#include "../src/math/CartesianToGeodeticFukushima.hpp"
#include "../src/math/CoordinateTransform.hpp"
#include "../src/utils/Constants.hpp"

TEST_CASE("Center of the Earth Degenerate Cases of Cartesian To Geodetic Conversion Test") {
    double latitudeEps = 1e-9;
    double longitudeEps = 1e-9;
    double altitudeEps = 1e-6;

    unsigned int numberOfIterations = 1;
    CartesianToGeodeticFukushima cart2geo(numberOfIterations);

    // Begin center of earth test
    Position centerOfEarth(0, 0.0, 0.0, 0.0);

    Eigen::Vector3d centerOfEarthTRF;
    CoordinateTransform::getPositionECEF(centerOfEarthTRF, centerOfEarth);


    Position testCenterOfEarthPosition(0,0.0,0.0,0.0);
    cart2geo.ecefToLongitudeLatitudeElevation(centerOfEarthTRF, testCenterOfEarthPosition);

    REQUIRE(std::abs(testCenterOfEarthPosition.getLatitude() - centerOfEarth.getLatitude()) < latitudeEps);
    REQUIRE(std::abs(testCenterOfEarthPosition.getLongitude() - centerOfEarth.getLongitude()) < longitudeEps);
    REQUIRE(std::abs(testCenterOfEarthPosition.getEllipsoidalHeight() - centerOfEarth.getEllipsoidalHeight()) < altitudeEps);
}

TEST_CASE("Equator Degenerate Cases of Cartesian To Geodetic Conversion Test") {

    double longitudeEpsilon = 1e-12;
    double latitudeEpsilon = 1e-12;

    int longitudeIncrement = 1;

    bool testFail = false;

    Position p(0,0.0,0.0,0.0);
    Eigen::Vector3d pTRF;
    Position testPosition(0,0.0,0.0,0.0); // must be equal to p in order for test to pass

    unsigned int numberOfIterations = 1;
    CartesianToGeodeticFukushima cart2geo(numberOfIterations);

    for (int longitude = -180; longitude <= 180; longitude += longitudeIncrement) {
        if (testFail) {
            break;
        }

        p.setLatitude(0.0);
        p.setLongitude(longitude);
        p.setEllipsoidalHeight(0);

        CoordinateTransform::getPositionECEF(pTRF, p);

        cart2geo.ecefToLongitudeLatitudeElevation(pTRF, testPosition);

        if (std::abs(longitude - testPosition.getLongitude()) > longitudeEpsilon) {
            testFail = true;
        }

        if (std::abs(0.0 - testPosition.getLatitude()) > latitudeEpsilon) {
            testFail = true;
        }
    }

    if (testFail) {
        std::cout << "Equator degenerate case for Cartesian To Geodetic conversion test:" << std::endl;
        std::cout << "What is expected:" << std::endl;
        std::cout << p << std::endl;
        std::cout << "What is obtained:" << std::endl;
        std::cout << testPosition << std::endl;
    }

}

TEST_CASE("North and South Pole Degenerate Cases of Cartesian To Geodetic Conversion Test") {

    double latitudeEps = 1e-9;
    double longitudeEps = 1e-9;
    double altitudeEps = 1e-6;

    unsigned int numberOfIterations = 1;
    CartesianToGeodeticFukushima cart2geo(numberOfIterations);

    // Begin north pole test
    Position northPole(0, 90.0, 0.0, 0);

    Eigen::Vector3d northPoleTRF;
    CoordinateTransform::getPositionECEF(northPoleTRF, northPole);


    Position testNorthPolePosition(0,0.0,0.0,0.0);
    cart2geo.ecefToLongitudeLatitudeElevation(northPoleTRF, testNorthPolePosition);

    REQUIRE(std::abs(testNorthPolePosition.getLatitude() - northPole.getLatitude()) < latitudeEps);
    REQUIRE(std::abs(testNorthPolePosition.getLongitude() - northPole.getLongitude()) < longitudeEps);
    REQUIRE(std::abs(testNorthPolePosition.getEllipsoidalHeight() - northPole.getEllipsoidalHeight()) < altitudeEps);

    // Begin south pole test
    Position southPole(0, -90.0, 0.0, 0);

    Eigen::Vector3d southPoleTRF;
    CoordinateTransform::getPositionECEF(southPoleTRF, southPole);

    Position testSouthPolePosition(0,0.0,0.0,0.0);
    cart2geo.ecefToLongitudeLatitudeElevation(southPoleTRF, testSouthPolePosition);

    REQUIRE(std::abs(testSouthPolePosition.getLatitude() - southPole.getLatitude()) < latitudeEps);
    REQUIRE(std::abs(testSouthPolePosition.getLongitude() - southPole.getLongitude()) < longitudeEps);
    REQUIRE(std::abs(testSouthPolePosition.getEllipsoidalHeight() - southPole.getEllipsoidalHeight()) < altitudeEps);

}

TEST_CASE("Cartesian To Geodetic Conversion Test") {

    double longitudeEpsilon = 1e-13;
    double latitudeEpsilon = 1e-13;
    double heightEpsilon = 1e-7;

    std::cout << std::setprecision(15) << std::endl;

    unsigned int numberOfIterations = 2;
    CartesianToGeodeticFukushima cart2geo(numberOfIterations);

    int longitudeIncrement = 2;
    int latitudeIncrement = 2;
    int heightIncrement = 50;

    double a = M_PI_2;

    if (a > M_PI_2) {
        std::cout << "a > PI/2" << std::endl;
    }
    if (-a < -M_PI_2) {
        std::cout << "-a < -PI/2" << std::endl;
    }

    bool testFail = false;


    Position p(0,0.0,0.0,0.0);
    Eigen::Vector3d pTRF;
    Position testPosition(0,0.0,0.0,0.0);

    for (int longitude = -180; longitude <= 180; longitude += longitudeIncrement) {
        if (testFail) {
            break;
        }

        for (int latitude = -89; latitude <= 89; latitude += latitudeIncrement) {
            if (testFail) {
                break;
            }

            for (int height = -1000; height <= 1000; height += heightIncrement) {
                if (testFail) {
                    break;
                }

                p.setLatitude(latitude);
                p.setLongitude(longitude);
                p.setEllipsoidalHeight(height);

                CoordinateTransform::getPositionECEF(pTRF, p);

                cart2geo.ecefToLongitudeLatitudeElevation(pTRF, testPosition);

                if (std::abs(testPosition.getLatitude() - latitude) > latitudeEpsilon) {
                    std::cout << "Latitude mismatch for cartesian to geodetic conversion" << std::endl;
                    std::cout << testPosition.getLatitude() << " should be " << latitude << std::endl;
                    testFail = true;
                }

                if (std::abs(testPosition.getLongitude() - longitude) > longitudeEpsilon) {
                    std::cout << "Longitude mismatch for cartesian to geodetic conversion" << std::endl;
                    std::cout << testPosition.getLongitude() << " should be " << longitude << std::endl;
                    testFail = true;
                }

                if (std::abs(testPosition.getEllipsoidalHeight() - height) > heightEpsilon) {
                    std::cout << "Height mismatch for cartesian to geodetic conversion" << std::endl;
                    std::cout << testPosition.getEllipsoidalHeight() << " should be " << height << std::endl;
                    testFail = true;
                }
            } // height
        } // latitude
    } // longitude

    if (testFail) {
        std::cout << "Cartesian To Geodetic Conversion Test Failure:" << std::endl;
        std::cout << "Expected position:" << std::endl;
        std::cout << p << std::endl;
        std::cout << "What is obtained:" << std::endl;
        std::cout << testPosition << std::endl;
    }

    REQUIRE(!testFail);
}

TEST_CASE("test the ecef conversion to longitude latitude elevation")
{
    Position result(0,0.0,0.0,0.0);
    Eigen::Vector3d ecefPosition(0.0,0.0,0.0);
    CartesianToGeodeticFukushima convertTest(0);
    convertTest.ecefToLongitudeLatitudeElevation(ecefPosition,result);
    REQUIRE(abs(result.getLatitude()-(0.0*R2D))<1e-10);
    REQUIRE(abs(result.getLongitude()-(0.0*R2D))<1e-10);
    REQUIRE(abs(result.getEllipsoidalHeight()-(0.0))<1e-10);
    ecefPosition(0) = 0.0;
    ecefPosition(1) = 0.0;
    ecefPosition(2) = 1.0;
    convertTest.ecefToLongitudeLatitudeElevation(ecefPosition,result);
    REQUIRE(abs(result.getLatitude()-(M_PI_2*R2D))<1e-10);
    REQUIRE(abs(result.getLongitude()-(0.0*R2D))<1e-10);
    REQUIRE(abs(result.getEllipsoidalHeight()-(1-(a_wgs84*(std::sqrt(1-e2_wgs84)))))<1e-10);
    ecefPosition(0) = 0.0;
    ecefPosition(1) = 0.0;
    ecefPosition(2) = -1.0;
    convertTest.ecefToLongitudeLatitudeElevation(ecefPosition,result);
    REQUIRE(abs(result.getLatitude()-(-M_PI_2*R2D))<1e-10);
    REQUIRE(abs(result.getLongitude()-(0.0*R2D))<1e-10);
    REQUIRE(abs(result.getEllipsoidalHeight()-(1-(a_wgs84*(std::sqrt(1-e2_wgs84)))))<1e-10);
}

#endif /* CARTESIANTOGEODETICCONVERSIONTEST_HPP */

