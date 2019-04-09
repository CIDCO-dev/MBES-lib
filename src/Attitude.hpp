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
 *  \brief Classe d'altitude.
 */
class Attitude {
public:
    
    /**
     * Crée une altitude
     */

    Attitude(){};
    
    /**
     * Crée une altitude
     * 
     * @param microEpoch  nombre de micro-seconde depuis janvier 1970 (micro-seconde)
     * @param rollDegrees la valeur entre deux rollies (degrés)
     * @param pitchDegrees la valeur en degrés qui détermine comment le bâteau penche (degrés)
     * @param headingDegrees détermine en degrés la direction où le bâteau pointe (degrés)
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
     * Détruit l'altitude
     */
    
    ~Attitude() {
    };

    /**Retourne l'angle roll*/
    double getRoll()        { return roll;}
    
    /**Retourne l'angle roll en radian*/
    double getRollRadians() { return roll * D2R;}
    
    /**Retourne la valeur sinus de l'angle roll*/
    double getSr()     { return sr;}
    
    /**Retourne la valeur cosinus de l'angle roll */
    double getCr()     { return cr;}

    /**Retourne l'angle pitch*/
    double getPitch()        { return pitch;}
    
    /**Retourne l'angle pitch en radian*/
    double getPitchRadians() { return pitch * D2R;}

    /**Retourne la valeur sinus de l'angle pitch*/
    double getSp()     { return sp;}
    
    /**Retourne la valeur cosinus de l'angle pitch*/
    double getCp()     { return cp;}

    /**Retourne l'angle heading*/
    double getHeading()        { return heading;}
    
    /**Retourne l'angle heading en radian*/
    double getHeadingRadians() { return heading * D2R;}
    
    /**Retourne la valeur sinus de l'angle heading*/
    double getSh()     { return sh;}
    
    /**Retourne la valeur cosinus de l'angle heading*/
    double getCh()     { return ch;}

    /**
     * Change l'angle roll et ses valeur sinus et cosinus selon l'angle reçu en paramètre
     * 
     * @param roll nouvelle angle roll
     */
    void setRoll(double roll){
	this->roll = roll;
	sr=sin(roll*D2R);
        cr=cos(roll*D2R);
    }

    /**
     * Change l'angle pitch et ses valeur sinus et cosinus selon l'angle reçu en paramètre
     * 
     * @param pitch nouvelle angle pitch
     */
    void setPitch(double pitch){
	this->pitch=pitch; 
        sp=sin(pitch*D2R);
        cp=cos(pitch*D2R);
    }

    /**
     * Change l'angle heading et ses valeur sinus et cosinus selon l'angle reçu en paramètre
     * 
     * @param heading nouvelle angle heading
     */
    void setHeading(double heading){
	this->heading=heading;
        sh=sin(heading*D2R);
        ch=cos(heading*D2R);
    }

    /**Retourne le temps en micro-seconde de l'altitude*/
    uint64_t getTimestamp(){ return timestamp;}

    /**
     * Change la valeur timestamp pour la valeur temps reçu en paramètre
     *
     * @param microEpoch nouvelle valeur temps de timestamp
     */
    void setTimestamp(uint64_t microEpoch){
	this->timestamp = microEpoch;
    }

    /**
     * Retourne une chaine de caractère donnant les angles d'un altitude
     * 
     * @param os chaine de caractères dont il faut ajouter l'information
     * @param obj l'altitude que l'on cherche à avoir l'information
     */
    friend std::ostream& operator<<(std::ostream& os, const Attitude& obj) {
        return os << "Roll: " << obj.roll << std::endl << "Pitch: " << obj.pitch << std::endl << "Heading: " << obj.heading << std::endl;
    };

private:
    
    /**Valeur de temps en micro-seconde calculé depuis janvier 1970 (micro-seconde)*/
    uint64_t  timestamp;

    /**Un angle entre deux rollies (degrés)*/
    double    roll;    //in degrees
    
    /**Un angle qui détermine comment le bâteau penche (degrés)*/
    double    pitch;   //in degrees
    
    /**Un angle qui détermine en degrés la direction où le bâteau pointe (degrés)*/
    double    heading; //in degrees

    /*Trigonometry is stored to prevent redundant recalculations*/
    
    /**Valeur sinus de l'angle roll*/
    double sr;
    
    /**Valeur cosinus de l'angle roll*/
    double cr;

    /**Valeur sinus de l'angle pitch*/
    double sp;
    
    /**Valeur cosinus de l'angle pitch*/
    double cp;

    /**Valeur sinus de l'angle heading*/
    double sh;
    
    /**Valeur cosinus de l'angle heading*/
    double ch;
};

#endif /* ATTITUDE_HPP */

