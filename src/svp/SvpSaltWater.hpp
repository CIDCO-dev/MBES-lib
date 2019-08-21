/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   SvpSaltWater.hpp
 * Author: jordan
 *
 * Created on August 20, 2019, 3:23 PM
 */

#ifndef SVPSALTWATER_HPP
#define SVPSALTWATER_HPP

#include <vector>


class SvpSaltWater : public SvpSelectionStrategy {
private:
    SoundVelocityProfile * saltWaterSvp = NULL;

public:

    SvpSaltWater(){
        SoundVelocityProfile * saltWaterSvp = new SoundVelocityProfile();
        //TODO: set time/location?
        saltWaterSvp->add(0, 1520);
        saltWaterSvp->add(15000, 1520);
    }

    ~SvpSaltWater() {
        if(saltWaterSvp != NULL) {
            delete saltWaterSvp;
        }
    }
    
    SoundVelocityProfile * chooseSvp(Position & position, Ping & ping) {
        return saltWaterSvp;
    }
    
    void addSvp(SoundVelocityProfile * svp) {
        //don't add it, this is a constant SVP for saltwater
    }
};

#endif /* SVPSALTWATER_HPP */

