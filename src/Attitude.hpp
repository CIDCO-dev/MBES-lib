/*
 * Copyright 2017-2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   Attitude.hpp
 * Author: glm,jordan
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

    Attitude(){};

    Attitude(uint64_t microEpoch,double rollDegrees,double pitchDegrees,double headingDegrees) :
	    timestamp(microEpoch),
	    roll(rollDegrees),
	    pitch(pitchDegrees),
	    heading(headingDegrees),
	    sr(sin(roll*D2R)),
	    cr(cos(roll*D2R)),
	    sp(sin(pitch*D2R)),
	    cp(cos(pitch*D2R)),
	    sh(sin(heading*D2R)),
	    ch(cos(heading*D2R))
    {};

    ~Attitude() {
    };

    double getRoll()   { return roll;}
    double getSr()     { return sr;}
    double getCr()     { return cr;}

    double getPitch()  { return pitch;}
    double getSp()     { return sp;}
    double getCp()     { return cp;}

    double getHeading(){ return heading;}
    double getSh()     { return sh;}
    double getCh()     { return ch;}

    void setRoll(double roll){
	this->roll = roll;
	sr=sin(roll*D2R);
        cr=cos(roll*D2R);
    }

    void setPitch(double pitch){
	this->pitch=pitch; 
        sp=sin(pitch*D2R);
        cp=cos(pitch*D2R);
    }

    void setHeading(double heading){
	this->heading=heading;
        sh=sin(heading*D2R);
        ch=cos(heading*D2R);
    }

    uint64_t getTimestamp(){ return timestamp;}

    void setTimestamp(uint64_t microEpoch){
	this->timestamp = microEpoch;
    }

    friend std::ostream& operator<<(std::ostream& os, const Attitude& obj) {
        return os << "Roll: " << obj.roll << std::endl << "Pitch: " << obj.pitch << std::endl << "Heading: " << obj.heading << std::endl;
    };

private:
    uint64_t  timestamp;
    double    roll;
    double    pitch;
    double    heading;

    /*Trigonometry is stored to prevent redundant recalculations*/
    double sr;
    double cr;

    double sp;
    double cp;

    double sh;
    double ch;
};

#endif /* ATTITUDE_HPP */

