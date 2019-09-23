/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   SvpNearestByTime.hpp
 * Author: jordan
 */

#ifndef SVPNEARESTBYTIME_HPP
#define SVPNEARESTBYTIME_HPP


#include <vector>
#include "SvpSelectionStrategy.hpp"
#include "SoundVelocityProfile.hpp"
#include "../utils/Exception.hpp"

#undef max
#undef min

class SvpNearestByTime : public SvpSelectionStrategy {
private:
    std::vector<SoundVelocityProfile*> svps;

public:

    SvpNearestByTime() {

    }

    ~SvpNearestByTime() {

    }

    void addSvp(SoundVelocityProfile * svp) {
        svps.push_back(svp);

    }

    SoundVelocityProfile * chooseSvp(Position & position, Ping & ping) {

        uint64_t dt = std::numeric_limits<uint64_t>::max();

        unsigned int indexNearest = 0;

        for (unsigned int i = 0; i < svps.size(); i++) {
            SoundVelocityProfile* svp = svps[i];

            if (svp->getTimestamp() == 0) {
                throw new Exception("Cannot apply SvpNearestByTime strategy to svp with timestamp==0");
            }

            //Warning: subtracting uint is dangerous!
            //check which is greater first
            uint64_t timeDiff = (ping.getTimestamp() > svp->getTimestamp()) ? ping.getTimestamp() - svp->getTimestamp() : svp->getTimestamp() - ping.getTimestamp();

            if (timeDiff < dt) {
                dt = timeDiff;
                indexNearest = i;
            }
        }

        return svps[indexNearest];
    }
};


#endif /* SVPNEARESTBYTIME_HPP */

