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
  static std::string julianTime(uint64_t microEpoch)
  {
    time_t date = microEpoch/1000000;
    struct tm * timeinfo;
    timeinfo = gmtime(&date);
    std::stringstream ssDate;
    ssDate << timeinfo->tm_year + 1900 << "-" << timeinfo->tm_yday + 1 << " " << timeinfo->tm_hour << ":" << timeinfo->tm_min << ":" << timeinfo->tm_sec;
    return ssDate.str();
  }

  /**
  * Returns the latitude in text form
  *
  * @param value the latitude
  */
  std::string latFormat(double value)
  {
    std::string direction;
    if (value<0)
    {
      direction = "South";
    }
    else
    {
      direction = "North";
    }
    return latlongFormat(value,direction);
  }

  /**
  * Returns the longitude in text form
  *
  * @param value the longitude
  */
  std::string longFormat(double value)
  {
    std::string direction;
    if (value<0)
    {
      direction = "West";
    }
    else
    {
      direction = "East";
    }
    return latlongFormat(value,direction);
  }

  /**
  * Returns the latitude or the longitude in format dd:mm:ss
  *
  * @param value the latitude or longitude to convert
  * @param direction direction(N,W,E,S) of the latitude or longitude
  */
  std::string latlongFormat(double value, std::string direction)
  {
    std::stringstream ss;
    value = std::abs(value);
    double degrees = std::trunc(value);
    value = (value - degrees) * 60;
    double minutes = std::trunc(value);
    double second = (value - minutes) * 60;
    ss << direction << " " << degrees << ":" << minutes << ":" << second;
    return ss.str();
  }

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
  bool readSectionHeader(std::string & row, uint64_t &nbrM,double &lat, double &lon)
  {
    int latdegrees;
    int latminute;
    int latsecond;
    int londegrees;
    int lonminute;
    int lonsecond;
    int year;
    int yday;
    int hour;
    int minute;
    int second;
    if (std::sscanf(row.c_str(), "Section %d-%d %d:%d:%d %d:%d:%d %d:%d:%d", &year,&yday,&hour,&minute,&second,&latdegrees,&latminute,&latsecond,&londegrees,&lonminute,&lonsecond)==11){
            lat = (double)latsecond/(double)60 + (double)latminute;
            lat = (double)lat/(double)60 + (double)latdegrees;
            lon = (double)lonsecond/(double)60 + (double)lonminute;
            lon = (double)lon/(double)60 + (double)londegrees;
            year = year-1970;
            yday = yday-1;
            nbrM = nbrM+year;
            nbrM = nbrM*365 + yday;
            int y = year+2;
            while (y >= 4) //?
            {
                y = y-4;
                nbrM = nbrM+1;
            }
            nbrM = nbrM*24 + hour;
            nbrM = nbrM*60 + minute;
            nbrM = nbrM*60 + second;
            nbrM = nbrM*1000000;
            return true;
    }

    return false;
  }
  /**
  * Adds a new value in the vector depths and speeds
  *
  * @param depth value to add in depths
  * @param soundSpeed value to add in speeds
  */
  void     add(double depth,double soundSpeed);

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

SoundVelocityProfile::SoundVelocityProfile() {
  longitude = latitude = nan("");
};

SoundVelocityProfile::~SoundVelocityProfile() {
};

void SoundVelocityProfile::add (double depth,double soundSpeed){
  samples.push_back(std::make_pair(depth,soundSpeed));
}

void SoundVelocityProfile::write(std::string & filename){
  std::ofstream out;
  out.open(filename);
  if(out.is_open()){
    //TODO: write proper date and lat/lon
    std::string sDate;
    sDate = julianTime(microEpoch);
    std::string slat;
    slat = latFormat(latitude);
    std::string slong;
    slong = longFormat(longitude);
    out << "[SVP_VERSION_2]" << "\r\n";
    out << filename << " \r\n";
    out << "Section " << sDate << " " << slat << " " << slong << " \r\n" ;//FIXME: put date as yyyy-ddd hh:mm:ss dd:mm:ss (lat) dd:mm:ss (lon)
    for(unsigned int i=0;i<samples.size();i++){
      out << samples[i].first << " " << samples[i].second << " \r\n";
    }
    out.close();
  }
}

bool SoundVelocityProfile::read(std::string & filename)
{
  std::string row;
  std::string cont;

  std::ifstream inFile(filename);

  if(inFile)
  {
    samples.clear();

    int i = 0;
    while (std::getline(inFile,row)){
      if (i == 0){
        int ver;
        if (std::sscanf(row.c_str(),"[SVP_VERSION_%d]",&ver)!=1){
	  std::cerr << "Wrong SVP file header" << std::endl;
	  inFile.close();
          return false;
        }
      }
      else if (i==1){
        std::string name;
        name = row.substr(0,row.find(" "));
	//Dont care about name, it couldve been renamed
      }
      else if (i==2){
        if(!readSectionHeader(row,microEpoch,latitude,longitude)){
	  std::cerr << "Bad section label" << std::endl;
          inFile.close();
	  return false;
        }
      }
      else{
        double deph;
        double speed;
        if (std::sscanf(row.c_str(), "%lf %lf",&deph,&speed)==2){
          add(deph,speed);
        }
        else{
          //ignore bad lines
        }
      }

      i++;
    }

    inFile.close();
    return true;
  }

  return false;
}

Eigen::VectorXd & SoundVelocityProfile::getDepths(){
  //lazy load internal vector
  if((unsigned int)depths.size() != samples.size()){
    depths.resize(samples.size());

    for(unsigned int i = 0;i<samples.size();i++){
      depths(i)=samples[i].first;
    }
  }

  return depths;
}

Eigen::VectorXd & SoundVelocityProfile::getSpeeds(){
  //lazy load internal vector
  if((unsigned int)speeds.size() != samples.size()){
    speeds.resize(samples.size());

    for(unsigned int i=0;i<samples.size();i++){
      speeds(i)=samples[i].second;
    }
  }

  return speeds;
}

#endif /* SOUNDVELOCITYPROFILE_HPP */
