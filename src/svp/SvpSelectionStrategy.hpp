/*
 * Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   SvpSelectionStrategy.hpp
 * Author: jordan
 */

#ifndef SVPSELECTIONSTRATEGY_HPP
#define SVPSELECTIONSTRATEGY_HPP

#include <vector>
#include "SoundVelocityProfile.hpp"
#include "../Position.hpp"
#include "../Ping.hpp"

class SvpSelectionStrategy {
public:    
    virtual SoundVelocityProfile * chooseSvp(Position & position, Ping & ping)=0;
    virtual void addSvp(SoundVelocityProfile * svp)=0;
};


#endif /* SVPSELECTIONSTRATEGY_HPP */

