/*
 * Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

#ifndef SOUNDVELOCITYPROFILEFACTORY_HPP
#define SOUNDVELOCITYPROFILEFACTORY_HPP

#include "SoundVelocityProfile.hpp"
#include <limits>
#include <vector>
#include <cmath>
#include "../utils/Constants.hpp"

/*!
 * \brief SoundVelocityProfile factory class
 * \author Guillaume Labbe-Morissette
 */
class SoundVelocityProfileFactory {
public:

    /**
     * Returns a SoundVelocityProfile model with salt water
     */
    static SoundVelocityProfile * buildSaltWaterModel() {
        SoundVelocityProfile * svp = new SoundVelocityProfile();
        //TODO: set time/location?
        svp->add(0, 1520);
        svp->add(15000, 1520);
        return svp;
    }

    /**
     * Returns a SoundVelocityProfile model with fresh water
     */
    static SoundVelocityProfile * buildFreshWaterModel() {
        SoundVelocityProfile * svp = new SoundVelocityProfile();
        //TODO: set time/location?
        svp->add(0, 1480);
        svp->add(15000, 1480);
        return svp;
    }

    /**
     * Returns nearest SoundVelocityProfile in time
     */
    static SoundVelocityProfile * selectNearestInTime(std::vector<SoundVelocityProfile*> svps, uint64_t referenceTimestamp) {
        uint64_t dt = std::numeric_limits<uint64_t>::max();

        unsigned int indexNearest = 0;

        for (unsigned int i = 0; i < svps.size(); ++i) {
            SoundVelocityProfile* svp = svps[i];

            if (std::abs(svp->getTimestamp() - referenceTimestamp) < dt) {
                dt = std::abs(svp->getTimestamp() - referenceTimestamp);
                indexNearest = i;
            }
        }

        return svps[indexNearest];
    }

    /**
     * Returns nearest SoundVelocityProfile in location
     */
    static SoundVelocityProfile * selectNearestInLocation(std::vector<SoundVelocityProfile*> svps, double referenceLatitude, double referenceLongitude) {
        double d = std::numeric_limits<double>::max();

        unsigned int indexNearest = 0;

        for (unsigned int i = 0; i < svps.size(); ++i) {
            SoundVelocityProfile* svp = svps[i];

            double dlat = svp->getLatitude() * D2R - referenceLatitude * D2R;
            double dlon = svp->getLongitude() * D2R - referenceLongitude * D2R;
            double distance = 2 * 63781370 * asin(sqrt(sin(dlat / 2) * sin(dlat / 2) + cos(referenceLatitude * D2R) * cos(svp->getLatitude() * D2R) * sin(dlon / 2) * sin(dlon / 2)));

            if (distance < d) {
                d = distance;
                indexNearest = i;
            }
        }

        return svps[indexNearest];
    }
};


#endif
