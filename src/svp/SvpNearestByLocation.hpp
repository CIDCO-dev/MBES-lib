/*
 * Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   SvpNearestByLocation.hpp
 * Author: jordan
 */

#ifndef SVPNEARESTBYLOCATION_HPP
#define SVPNEARESTBYLOCATION_HPP

#include "SoundVelocityProfile.hpp"
#include "SvpSelectionStrategy.hpp"

class SvpNearestByLocation : public SvpSelectionStrategy {
private:
    std::vector<SoundVelocityProfile*> svps;

public:
    SvpNearestByLocation() {
    }

    ~SvpNearestByLocation() {

    }
    
    void addSvp(SoundVelocityProfile * svp) {
        svps.push_back(svp);
    }

    SoundVelocityProfile * chooseSvp(Position & position, Ping & ping) {
        double d = std::numeric_limits<double>::max();

        unsigned int indexNearest = 0;


        for (unsigned int i = 0; i < svps.size(); ++i) {
            SoundVelocityProfile* svp = svps[i];

            double dlat = svp->getLatitude() * D2R - position.getLatitude() * D2R;
            double dlon = svp->getLongitude() * D2R - position.getLongitude() * D2R;
            double distance = 2 * 63781370 * asin(sqrt(sin(dlat / 2) * sin(dlat / 2) + cos(position.getLatitude() * D2R) * cos(svp->getLatitude() * D2R) * sin(dlon / 2) * sin(dlon / 2)));

            if (distance < d) {
                d = distance;
                indexNearest = i;
            }
        }

        return svps[indexNearest];
    }
};

#endif /* SVPNEARESTBYLOCATION_HPP */

