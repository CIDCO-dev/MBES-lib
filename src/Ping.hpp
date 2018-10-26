/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   Ping.hpp
 * Author: jordan
 *
 * Created on September 14, 2018, 10:10 AM
 */

#ifndef PING_HPP
#define PING_HPP

#include <iostream>
#include <Eigen/Dense>
#include "Attitude.hpp"
#include "Position.hpp"

class Ping {

public:
    const double transmissionTimestamp;
    const Position* transmissionPosition = NULL;
    const Attitude* transmissionAttitude = NULL;

    const double receptionTimestamp;
    const Position* receptionPosition = NULL;
    const Attitude* receptionAttitude = NULL;

    const double surfaceSoundSpeed;
    const double twoWayTravelTime;
    const double alongTrackAngle; // emission angle
    const double acrossTrackAngle; // reception angle

    /*Trigonometry is stored to prevent redundant recalculations*/
    const double sA; // alpha = kappa = alongTrackAngle = emission angle = tilt angle
    const double cA;

    const double sB; // beta = zeta = acrossTrackAngle = reception angle = beam angle
    const double cB;

    Ping(const double transmissionTimestamp,
            const Position* transmissionPosition,
            const Attitude* transmissionAttitude,
            const double receptionTimestamp,
            const Position* receptionPosition,
            const Attitude* receptionAttitude,
            const double surfaceSoundSpeed,
            const double twoWayTravelTime,
            const double alongTrackAngle,
            const double acrossTrackAngle) :
    transmissionTimestamp(transmissionTimestamp),
    transmissionPosition(transmissionPosition),
    transmissionAttitude(transmissionAttitude),
    receptionTimestamp(receptionTimestamp),
    receptionPosition(receptionPosition),
    receptionAttitude(receptionAttitude),
    surfaceSoundSpeed(surfaceSoundSpeed),
    twoWayTravelTime(twoWayTravelTime),
    alongTrackAngle(alongTrackAngle),
    acrossTrackAngle(acrossTrackAngle),
    sA(sin(alongTrackAngle*D2R)),
    cA(cos(alongTrackAngle*D2R)),
    sB(sin(acrossTrackAngle*D2R)),
    cB(cos(acrossTrackAngle*D2R)){
    };

    ~Ping() {
        if (transmissionAttitude) {
            delete transmissionAttitude;
        }

        if (transmissionPosition) {
            delete transmissionPosition;
        }

        if (receptionAttitude) {
            delete receptionAttitude;
        }

        if (receptionPosition) {
            delete receptionPosition;
        }
    };

    static Ping* build(
            double transmissionTimestamp,
            double transmissionLatitude,
            double transmissionLongitude,
            double transmissionEllipsoidHeight,
            double transmissionRoll,
            double transmissionPitch,
            double transmissionHeading,
            double receptionTimestamp,
            double receptionLatitude,
            double receptionLongitude,
            double receptionEllipsoidHeight,
            double receptionRoll,
            double receptionPitch,
            double receptionHeading,
            double surfaceSoundSpeed,
            double twoWayTravelTime,
            double alongTrackAngle,
            double acrossTrackAngle) {

        return new Ping(transmissionTimestamp,
                new Position(transmissionLatitude, transmissionLongitude, transmissionEllipsoidHeight),
                new Attitude(transmissionRoll, transmissionPitch, transmissionHeading),
                receptionTimestamp,
                new Position(receptionLatitude, receptionLongitude, receptionEllipsoidHeight),
                new Attitude(receptionRoll, receptionPitch, receptionHeading),
                surfaceSoundSpeed,
                twoWayTravelTime,
                alongTrackAngle,
                acrossTrackAngle);
    };

    friend std::ostream& operator<<(std::ostream& os, const Ping& obj) {

        return os << "Transmission timestamp: " << obj.transmissionTimestamp << std::endl
                << "Transmission position: " << std::endl << *obj.transmissionPosition
                << "Transmission attitude: " << std::endl << *obj.transmissionAttitude
                << "Reception timestamp: " << obj.receptionTimestamp << std::endl
                << "Reception position: " << std::endl << *obj.receptionPosition
                << "Reception attitude: " << std::endl << *obj.receptionAttitude
                << "Surface speed of sound: " << obj.surfaceSoundSpeed << std::endl
                << "Two way travel time: " << obj.twoWayTravelTime << std::endl
                << "Along track angle: " << obj.alongTrackAngle << std::endl
                << "Across track angle: " << obj.acrossTrackAngle << std::endl;
    };
};


#endif /* PING_HPP */

