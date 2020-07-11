/*
 * Copyright 2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   CartesianToGeodeticFukushima.hpp
 * Author: jordan
 */

#ifndef CARTESIANTOGEODETICFUKUSHIMA_HPP
#define CARTESIANTOGEODETICFUKUSHIMA_HPP

#ifdef _WIN32
#define _USE_MATH_DEFINES
#include <math.h>
#else
#include <cmath>
#endif

#include <vector>
#include <Eigen/Dense>
#include "../Position.hpp"
#include "../utils/Constants.hpp"

class CartesianToGeodeticFukushima {
    /*
     * This class implements the method given in Fukushima (2006):
     *
     * Transformation from Cartesian to geodetic coordinates
     * accelerated by Halley's method
     * DOI: 10.1007/s00190-006-0023-2
     */
private:

    unsigned int numberOfIterations;

    // Ellipsoid parameters
    double a; // semi-major axis
    double e2; // first eccentricity squared

    // Derived parameters
    double b; // semi-minor axis
    double a_inverse; // 1/a
    double ec; // sqrt(1 - e*e)

public:

    CartesianToGeodeticFukushima(unsigned int numberOfIterations, double a=a_wgs84, double e2=e2_wgs84) :
    numberOfIterations(numberOfIterations), a(a), e2(e2) {
        ec = std::sqrt(1 - e2);
        b = a*ec;
        a_inverse = 1 / a;
    }

    ~CartesianToGeodeticFukushima() {};

    void ecefToLongitudeLatitudeElevation(Eigen::Vector3d & ecefPosition, Position & positionGeographic) {
        double x = ecefPosition(0);
        double y = ecefPosition(1);
        double z = ecefPosition(2);

        // Center of the Earth
        if (x == 0.0 && y == 0.0 && z == 0.0) {
            positionGeographic.setLatitude(0.0);
            positionGeographic.setLongitude(0.0);
            positionGeographic.setEllipsoidalHeight(0.0);
            return;
        }

        // Position at Poles
        if (x == 0.0 && y == 0.0 && z != 0.0) {
            if (z > 0) {
                positionGeographic.setLatitude(M_PI_2*R2D);
            } else {
                positionGeographic.setLatitude(-M_PI_2*R2D);
            }

            positionGeographic.setLongitude(0.0);
            positionGeographic.setEllipsoidalHeight(std::abs(z) - b);
            return;
        }
        
        double pp = x * x + y * y;
        double p = std::sqrt(pp);

        // Position at Equator
        if (z == 0.0) {
            positionGeographic.setLatitude(0.0);
            positionGeographic.setLongitude(estimateLongitude(x, y, p)*R2D);
            positionGeographic.setEllipsoidalHeight(std::sqrt(x * x + y * y) - a);
            return;
        }

        double P = p*a_inverse;
        double Z = a_inverse * ec * std::abs(z);

        //double R = std::sqrt(pp + z * z);
        std::vector<double> S(numberOfIterations + 1, 0);
        std::vector<double> C(numberOfIterations + 1, 0);

        std::vector<double> D(numberOfIterations + 1, 0);
        std::vector<double> F(numberOfIterations + 1, 0);

        std::vector<double> A(numberOfIterations + 1, 0);
        std::vector<double> B(numberOfIterations + 1, 0);

        S[0] = Z; //starter variables. See (Fukushima, 2006) p.691 equation (17)
        C[0] = ec*P; //starter variables. See (Fukushima, 2006) p.691 equation (17)
        A[0] = std::sqrt(S[0] * S[0] + C[0] * C[0]);
        B[0] = 1.5 * e2 * e2 * P * S[0] * S[0] * C[0] * C[0] * (A[0] - ec); //starter variables. See (Fukushima, 2006) p.691 equation  (18)

        unsigned int iterationNumber = 1;

        while (iterationNumber <= numberOfIterations) {

            D[iterationNumber - 1] =
                    Z * A[iterationNumber - 1] * A[iterationNumber - 1] * A[iterationNumber - 1] +
                    e2 * S[iterationNumber - 1] * S[iterationNumber - 1] * S[iterationNumber - 1];
            F[iterationNumber - 1] =
                    P * A[iterationNumber - 1] * A[iterationNumber - 1] * A[iterationNumber - 1] -
                    e2 * C[iterationNumber - 1] * C[iterationNumber - 1] * C[iterationNumber - 1];

            S[iterationNumber] =
                    D[iterationNumber - 1] * F[iterationNumber - 1] -
                    B[iterationNumber - 1] * S[iterationNumber - 1];
            C[iterationNumber] =
                    F[iterationNumber - 1] * F[iterationNumber - 1] -
                    B[iterationNumber - 1] * C[iterationNumber - 1];

            A[iterationNumber] = std::sqrt(
                    S[iterationNumber] * S[iterationNumber] +
                    C[iterationNumber] * C[iterationNumber]);

            B[iterationNumber] =
                    1.5 *
                    e2 * S[iterationNumber] *
                    C[iterationNumber] * C[iterationNumber] *
                    ((P * S[iterationNumber] - Z * C[iterationNumber]) * A[iterationNumber] -
                    e2 * S[iterationNumber] * C[iterationNumber]);

            ++iterationNumber;
        }

        double lon = estimateLongitude(x, y, p);

        double Cc = ec*C[numberOfIterations];
        double lat = estimateLatitude(z, S[numberOfIterations], Cc);
        double h = estimateHeight(z, p, A[numberOfIterations], S[numberOfIterations], Cc);

        positionGeographic.setLatitude(lat*R2D);
        positionGeographic.setLongitude(lon*R2D);
        positionGeographic.setEllipsoidalHeight(h);
    }

    double estimateLongitude(double x, double y, double p) {
        // Vermeille (2004), stable longitude calculation
        // atan(y/x) suffers when x = 0

        if (y < 0) {
            return -M_PI_2 + 2*std::atan(x/(p - y));
        }

        return M_PI_2 - 2*std::atan(x/(p + y));
    }

    double estimateLatitude(double z, double S, double Cc) {
        double lat = std::abs(std::atan(S / Cc));

        if (z < 0) {
            return -lat;
        }

        return lat;
    }

    double estimateHeight(double z, double p, double A, double S, double Cc) {
        return (p * Cc + std::abs(z) * S - b * A) / std::sqrt(Cc * Cc + S * S);
    }
};

#endif /* CARTESIANTOGEODETICFUKUSHIMA_HPP */
