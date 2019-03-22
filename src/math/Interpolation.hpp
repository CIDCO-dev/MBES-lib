/*
 * Copyright 2017-2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   Interpolator.hpp
 * Author: glm,jordan
 *
 * Created on October 2, 2018, 3:04 PM
 */

#ifndef INTERPOLATOR_HPP
#define INTERPOLATOR_HPP

#include <stdexcept>
#include <cmath>

#include "../Position.hpp"
#include "../Attitude.hpp"

class Interpolator {
public:

    static Position* interpolatePosition(Position & p1, Position & p2, uint64_t timestamp) {
        double interpLat = linearInterpolation(p1.getLatitude(), p2.getLatitude(), timestamp, p1.getTimestamp(), p2.getTimestamp());
        double interpLon = linearInterpolation(p1.getLongitude(), p2.getLongitude(), timestamp, p1.getTimestamp(), p2.getTimestamp());
        double interpAlt = linearInterpolation(p1.getEllipsoidalHeight(), p2.getEllipsoidalHeight(), timestamp, p1.getTimestamp(), p2.getTimestamp());
        return new Position(timestamp,interpLat, interpLon, interpAlt);
    }

    static Attitude* interpolateAttitude(Attitude & a1, Attitude & a2,uint64_t timestamp) {
        double interpRoll = linearAngleInterpolation(a1.getRoll(), a2.getRoll(), timestamp, a1.getTimestamp(), a2.getTimestamp());
        double interpPitch = linearAngleInterpolation(a1.getPitch(), a2.getPitch(), timestamp, a1.getTimestamp(), a2.getTimestamp());
        double interpHeading = linearAngleInterpolation(a1.getHeading(), a2.getHeading(), timestamp, a1.getTimestamp(), a2.getTimestamp());
        return new Attitude(timestamp,interpRoll, interpPitch, interpHeading);
    }

    static double linearInterpolation(double y1, double y2, uint64_t x, uint64_t x1, uint64_t x2) {
        return y1 + (y2 - y1)*(x - x1) / (x2 - x1);
    }

    static double linearAngleInterpolation(double psi1, double psi2, uint64_t t, uint64_t t1, uint64_t t2) {

        if (psi1 < 0 || psi1 >= 360 || psi2 < 0 || psi2 >= 360) {
            throw std::invalid_argument("Angles need to be between 0 (inclusive) and 360 (exclusive) degrees");
        }

        if (psi1 == psi2) {
            return psi1;
        }

        double delta = (t - t1) / (t2 - t1);
        double dpsi = std::fmod((std::fmod(psi2 - psi1, 360) + 540), 360) - 180;
        double interpolation = psi1 + dpsi*delta;

        if(interpolation >= 0 && interpolation < 360) {
            return interpolation;
        }

        double moduloInterpolation = std::fmod(interpolation, 360);

        if(moduloInterpolation < 0) {
            return moduloInterpolation + 360;
        }

        return moduloInterpolation;
    }

    static double linearAngleRadiansInterpolation(double psi1, double psi2, uint64_t t, uint64_t t1, uint64_t t2) {
        double interpDegrees = linearAngleInterpolation(psi1*180.0/M_PI, psi2*180.0/M_PI, t, t1, t2);
        return interpDegrees*M_PI/180.0;
    }
};

#endif /* INTERPOLATOR_HPP */
