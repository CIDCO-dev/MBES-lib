/*
 * Copyright 2017-2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   Attitude.hpp
 * Author: EmileGagne, glm,jordan
 *
 * Created on September 13, 2018, 3:28 PM
 */

#ifndef ATTITUDE_HPP
#define ATTITUDE_HPP

#include <iostream>
#include "utils/Constants.hpp"
#include <cmath>

/*!
 *  \brief Attitude class.
 */
class Attitude {
public:
    
    /**
     * Create a attitude
     */

    Attitude(){};
    
    /**
     * Create a attitude
     * 
     * @param microEpoch  number of micro-second calculated since January 1970 (micro-second)
     * @param rollDegrees the value of the angle between two rolls (degrees)
     * @param pitchDegrees the value of the angle who determine how the boat is incline (degrees)
     * @param headingDegrees the value of the angle who determine where the boat is heading (degrees)
     */

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
    
    /**
     * Destroy the attitude 
     */
    
    ~Attitude() {
    };

    /**Return the angle roll*/
    double getRoll()        { return roll;}
    
    /**Return the radian angle of roll*/
    double getRollRadians() { return roll * D2R;}
    
    /**Return the sinus value of the angle roll*/
    double getSr()     { return sr;}
    
    /**Return the cosine value of the angle roll*/
    double getCr()     { return cr;}

    /**Return the angle pitch*/
    double getPitch()        { return pitch;}
    
    /**Return the radian angle of pitch*/
    double getPitchRadians() { return pitch * D2R;}

    /**Return the sinus value of the angle pitch*/
    double getSp()     { return sp;}
    
    /**Return the cosine value of the angle pitch*/
    double getCp()     { return cp;}

    /**Return the angle heading*/
    double getHeading()        { return heading;}
    
    /**Return the radian angle of heading*/
    double getHeadingRadians() { return heading * D2R;}
    
    /**Return the sinus value of the angle heading*/
    double getSh()     { return sh;}
    
    /**Return the cosine value of the angle heading*/
    double getCh()     { return ch;}

    /**
     * Change the angle roll and his values sinus and cosine
     * 
     * @param roll the new angle roll
     */
    void setRoll(double roll){
	this->roll = roll;
	sr=sin(roll*D2R);
        cr=cos(roll*D2R);
    }

    /**
     * Change the angle pitch and his values sinus and cosine
     * 
     * @param pitch the new angle pitch
     */
    void setPitch(double pitch){
	this->pitch=pitch; 
        sp=sin(pitch*D2R);
        cp=cos(pitch*D2R);
    }

    /**
     * Change the angle heading and his values sinus and cosine
     * 
     * @param heading the new angle heading
     */
    void setHeading(double heading){
	this->heading=heading;
        sh=sin(heading*D2R);
        ch=cos(heading*D2R);
    }

    /**Return the time stamp of the attitude*/
    uint64_t getTimestamp(){ return timestamp;}

    /**
     * Change the value timestamp
     *
     * @param microEpoch the new timestamp
     */
    void setTimestamp(uint64_t microEpoch){
	this->timestamp = microEpoch;
    }

    /**
     * Return a text value with the informations of the attitude
     * 
     * @param os text value who most contain the information
     * @param obj the attitude that we need to get the information
     */
    friend std::ostream& operator<<(std::ostream& os, const Attitude& obj) {
        return os << "Roll: " << obj.roll << std::endl << "Pitch: " << obj.pitch << std::endl << "Heading: " << obj.heading << std::endl;
    };

private:
    
    /**Number of micro-second calculated since January 1970 (micro-second)*/
    uint64_t  timestamp;

    /**Value of the angle between two rolls (degrees)*/
    double    roll;    //in degrees
    
    /**Value of the angle who determine how the boat is incline (degrees)*/
    double    pitch;   //in degrees
    
    /**Value of the angle who determine where the boat is heading (degrees)*/
    double    heading; //in degrees

    /*Trigonometry is stored to prevent redundant recalculations*/
    
    /**Sinus value of the angle roll*/
    double sr;
    
    /**Cosine value of the angle roll*/
    double cr;

    /**Sinus value of the angle pitch*/
    double sp;
    
    /**Cosine value of the angle pitch*/
    double cp;

    /**Sinus value of the angle heading*/
    double sh;
    
    /**Cosine value of the angle heading*/
    double ch;
};

#endif /* ATTITUDE_HPP */

