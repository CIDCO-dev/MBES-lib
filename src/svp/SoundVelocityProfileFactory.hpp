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
#include "../utils/Exception.hpp"

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
};


#endif
