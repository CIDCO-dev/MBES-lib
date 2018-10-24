/*
 * Copyright 2018 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   Swath.hpp
 * Author: jordan,glm
 *
 * Created on September 14, 2018, 2:39 PM
 */

#ifndef SWATH_HPP
#define SWATH_HPP

#include <iostream>
#include <vector>
#include "Ping.hpp"

class Swath {
public:

    const std::vector<Ping *>* pings = NULL;
    const unsigned int beamCount;

    Swath(std::vector<Ping*>* pings) : pings(pings), beamCount(pings->size()) {
    }

    ~Swath() {
        for(unsigned int i=0; i<beamCount; i++) {
            delete (*pings)[i];
        }

        delete pings;
    };

    friend std::ostream& operator<<(std::ostream& os, const Swath& obj) {
        for(unsigned int i=0; i<obj.beamCount; i++) {
            os << *((*obj.pings)[i]) << std::endl;
        }
        return os;
    }

};

#endif /* SWATH_HPP */

