/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/
/* 
 * File:   SidescanPing.cpp
 * Author: glm, jordan
 * 
 * Created on August 28, 2019, 5:34 PM
 */

#include "SidescanPing.hpp"

SidescanPing::SidescanPing() : attitude(NULL), position(NULL) {
}

SidescanPing::SidescanPing(const SidescanPing& orig) {
}

SidescanPing::~SidescanPing() {
    if(attitude) delete attitude;
    if(position) delete position;
}

