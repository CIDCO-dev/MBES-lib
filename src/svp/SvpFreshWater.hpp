/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   SvpFreshWater.hpp
 * Author: jordan
 */

#ifndef SVPFRESHWATER_HPP
#define SVPFRESHWATER_HPP

class SvpFreshWater : public SvpSelectionStrategy {
private:
    SoundVelocityProfile * freshWaterSvp = NULL;

public:

    SvpFreshWater(){
        SoundVelocityProfile * saltWaterSvp = new SoundVelocityProfile();
        //TODO: set time/location?
        saltWaterSvp->add(0, 1480);
        saltWaterSvp->add(15000, 1480);
    }

    ~SvpFreshWater() {
        if(freshWaterSvp != NULL) {
            delete freshWaterSvp;
        }
    }
    
    SoundVelocityProfile * chooseSvp(Position & position, Ping & ping) {
        return freshWaterSvp;
    }
    
    void addSvp(SoundVelocityProfile * svp) {
        //don't add it, this is a constant SVP for freshwater
    }
};

#endif /* SVPFRESHWATER_HPP */

