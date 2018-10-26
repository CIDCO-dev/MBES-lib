/*
 * Copyright 2018 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   Attitude.hpp
 * Author: jordan
 *
 * Created on September 13, 2018, 3:28 PM
 */

#ifndef ATTITUDE_HPP
#define ATTITUDE_HPP

#include <iostream>
#include "utils/Constants.hpp"
#include <cmath>

class Attitude {
public:

    const double roll;
    const double pitch;
    const double heading;

    /*Trigonometry is stored to prevent redundant recalculations*/
    const double sr;
    const double cr;
    const double sp;
    const double cp;
    const double sh;
    const double ch;

    Attitude(const double roll, const double pitch, const double heading) :
    roll(roll),
    pitch(pitch),
    heading(heading),
    sr(sin(roll*D2R)),
    cr(cos(roll*D2R)),
    sp(sin(pitch*D2R)),
    cp(cos(pitch*D2R)),
    sh(sin(heading*D2R)),
    ch(cos(heading*D2R)) {
    };

    ~Attitude() {
    };

    friend std::ostream& operator<<(std::ostream& os, const Attitude& obj) {
        return os << "Roll: " << obj.roll << std::endl << "Pitch: " << obj.pitch << std::endl << "Heading: " << obj.heading << std::endl;
    };

};

#endif /* ATTITUDE_HPP */

