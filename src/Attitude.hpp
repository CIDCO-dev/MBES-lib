/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef ATTITUDE_HPP
#define ATTITUDE_HPP

#include <iostream>
#include "utils/Constants.hpp"
#include <cmath>

/*!
* \brief Attitude class.
* \author Guillaume Labbe-Morissette, Jordan McManus, Emile Gagne
* \date September 13, 2018, 3:28 PM
*/
class Attitude {
public:

  /**
  * Creates an attitude
  */
  Attitude(){};

  /**
  * Creates an attitude
  *
  * @param microEpoch number of micro-seconds since January 1st 1970
  * @param rollDegrees the roll angle of the boat in degrees w.r.t the navigation frame
  * @param pitchDegrees the angle value of the pitch movement of the boat (degrees)
  * @param headingDegrees the angle value of the yaw movement of the boat (degrees)
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
  * Destroys the attitude
  */
  ~Attitude() {
  };

  /**Returns the roll angle*/
  double getRoll()        { return roll;}

  /**Returns the radian of the roll angle*/
  double getRollRadians() { return roll * D2R;}

  /**Returns the sine value of the roll angle*/
  double getSr()     { return sr;}

  /**Returns the cosine value of the roll angle*/
  double getCr()     { return cr;}

  /**Returns the pitch angle*/
  double getPitch()        { return pitch;}

  /**Returns the radian the pitch angle*/
  double getPitchRadians() { return pitch * D2R;}

  /**Returns the sine value of the pitch angle*/
  double getSp()     { return sp;}

  /**Returns the cosine value of the pitch angle*/
  double getCp()     { return cp;}

  /**Returns the heading angle*/
  double getHeading()        { return heading;}

  /**Returns the radian of the heading angle*/
  double getHeadingRadians() { return heading * D2R;}

  /**Returns the sine value of the heading angle*/
  double getSh()     { return sh;}

  /**Returns the cosine value of the heading angle*/
  double getCh()     { return ch;}

  /**
  * Sets the roll angle and his values sine and cosine
  *
  * @param roll the new roll angle
  */
  void setRoll(double roll){
    this->roll = roll;
    sr=sin(roll*D2R);
    cr=cos(roll*D2R);
  }

  /**
  * Sets the pitch angle and its sine and cosine
  *
  * @param pitch the new pitch angle
  */
  void setPitch(double pitch){
    this->pitch=pitch;
    sp=sin(pitch*D2R);
    cp=cos(pitch*D2R);
  }

  /**
  * Sets the heading angle and its sine and cosine
  *
  * @param heading the new heading angle
  */
  void setHeading(double heading){
    this->heading=heading;
    sh=sin(heading*D2R);
    ch=cos(heading*D2R);
  }

  /**Returns the timestamp of the attitude*/
  uint64_t getTimestamp(){ return timestamp;}

  /**
  * Sets the timestamp value
  *
  * @param microEpoch the new timestamp
  */
  void setTimestamp(uint64_t microEpoch){
    this->timestamp = microEpoch;
  }

  /**
  * Returns true is the first timestamp is smaller than the second
  *
  * @param a1 first Attitude
  * @param a2 second Attitude
  */
  static bool sortByTimestamp(Attitude & a1,Attitude & a2){
    return a1.getTimestamp() < a2.getTimestamp();
  }


  /**
  * Returns a text value with the informations of the attitude
  *
  * @param os text value that contains the most of the information
  * @param obj the Attitude that we need to get the information from
  */
  friend std::ostream& operator<<(std::ostream& os, const Attitude& obj) {
    return os << "Roll: " << obj.roll << std::endl << "Pitch: " << obj.pitch << std::endl << "Heading: " << obj.heading << std::endl;
  };

private:

  /**Number of micro-second calculated since January 1970 (micro-second)*/
  uint64_t  timestamp;

  /**The angle value of the roll movement of the boat (degrees)*/
  double    roll;    //in degrees

  /**The angle value of the pitch movement of the boat (degrees)*/
  double    pitch;   //in degrees

  /**The angle value of the yaw movement of the boat (degrees)*/
  double    heading; //in degrees

  /*Trigonometry is stored to prevent redundant recalculations*/

  /**Sine value of the roll angle*/
  double sr;

  /**Cosine value of the roll angle*/
  double cr;

  /**Sine value of the pitch angle*/
  double sp;

  /**Cosine value of the pitch angle*/
  double cp;

  /**Sine value of the heading angle*/
  double sh;

  /**Cosine value of the heading angle*/
  double ch;
};

#endif /* ATTITUDE_HPP */
