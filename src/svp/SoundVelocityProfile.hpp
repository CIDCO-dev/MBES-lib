/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef SOUNDVELOCITYPROFILE_HPP
#define SOUNDVELOCITYPROFILE_HPP

#include <list>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>
#include <ctime>
#include <string>
#include "../utils/TimeUtils.hpp"

/*!
* \brief SoundVelocityProfile class
* \author Guillaume Labbe-Morissette, Jordan McManus, Emile Gagne
* \date August 22, 2018, 10:30 AM
*/
class SoundVelocityProfile {
public:

  /**Creates a sound velocity*/
  SoundVelocityProfile();

  /**Destroys a sound velocity*/
  ~SoundVelocityProfile();


  /**
  * Writes the informations of the SoundVelocityProfile in a file
  *
  * @param filename the name of the file that will be written in
  */
  void write(std::string & filename);

  /**
  * Reads a file that contains the information of a SoundVelocityProfile\n\n
  * Returns true if the reading was successfully
  *
  * @param filename the name of the that will use to read
  */
  bool read(std::string & filename);

  /**Returns the size of the SoundVelocityProfile*/
  unsigned int getSize() {
    return getDepths().size();
  };

  /**Returns the latitude of the SoundVelocityProfile*/
  double getLatitude() 	 { return latitude; }

  /**
  * Sets the latitude of the SoundVelocityProfile
  *
  * @param l the new latitude
  */
  void   setLatitude(double l) { latitude=l;}

  /**Returns the longitude of the SoundVelocityProfile*/
  double getLongitude()	 { return longitude;}

  /**
  * Sets the longitude of the SoundVelocityProfile
  *
  * @param l the new longitude of the SoundVelocityProfile
  */
  void   setLongitude(double l){ longitude=l;}

  /**Return the timestamp of the sound velocity*/
  uint64_t getTimestamp(){ return microEpoch;};

  /**
  * Sets the timestamp of the SoundVelocityProfile
  *
  * @param t the new timestamp
  */
  void     setTimestamp(uint64_t t) { microEpoch=t;};

  /**
  * Returns the timestamp in julian time format (yyyy-ddd hh:mm:ss)
  *
  * @param microEpoch Timestamp in micro-epoch seconds
  */
  std::string julianTime(uint64_t microEpoch);
  
  /**
  * Returns the latitude in text form
  *
  * @param value the latitude
  */
  std::string latFormat(double value);

  /**
  * Returns the longitude in text form
  *
  * @param value the longitude
  */
  std::string longFormat(double value);

  /**
  * Returns the latitude or the longitude in format dd:mm:ss
  *
  * @param value the latitude or longitude to convert
  * @param direction direction(N,W,E,S) of the latitude or longitude
  */
  std::string latlongFormat(double value, std::string direction);

  /**
  * Read a row of text that contains the timestamp, the latitude and the longitude of
  * the SoundVelocityProfile\n\n
  * Returns true if the reading was successfully
  *
  * @param row the row that must be read
  * @param nbrM value of the timestamp we get after the reading
  * @param lat value of the latitude we get after the reading
  * @param lon value of the longitude we get after the reading
  */
  bool readSectionHeader(std::string & row, uint64_t &nbrM,double &lat, double &lon);
  
  /**
  * Adds a new value in the vector depths and speeds
  *
  * @param depth value to add in depths
  * @param soundSpeed value to add in speeds
  */
  void add(double depth,double soundSpeed);

  /**Returns the depths vector*/
  Eigen::VectorXd & getDepths();

  /**Returns the speeds vector*/
  Eigen::VectorXd & getSpeeds();

private:

  /**timestamp value of the SoundVelocityProfile (micro-second)*/
  uint64_t  microEpoch; //timestamp

  /**latitude value of the SoundVelocityProfile*/
  double latitude;

  /**longitude value of the SoundVelocityProfile*/
  double longitude;

  /**vector that contains the dephts of the SoundVelocityProfile*/
  Eigen::VectorXd depths;

  /**vector that contain the speeds of the SoundVelocityProfile*/
  Eigen::VectorXd speeds;

  /**vector that contain the depths and the speeds*/
  std::vector<std::pair<double,double>> samples;
};


#endif /* SOUNDVELOCITYPROFILE_HPP */
