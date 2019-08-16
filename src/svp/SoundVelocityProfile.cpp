/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef SOUNDVELOCITYPROFILE_CPP
#define SOUNDVELOCITYPROFILE_CPP

#include "SoundVelocityProfile.hpp"

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

/**
* Returns the latitude in text form
*
* @param value the latitude
*/
std::string SoundVelocityProfile::latFormat(double value)
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
std::string SoundVelocityProfile::longFormat(double value)
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
* Returns the timestamp in julian time format (yyyy-ddd hh:mm:ss)
*
* @param microEpoch Timestamp in micro-epoch seconds
*/
std::string SoundVelocityProfile::julianTime(uint64_t microEpoch)
{
    time_t date = microEpoch/1000000;
    struct tm * timeinfo;
    timeinfo = gmtime(&date);
    std::stringstream ssDate;
    ssDate << timeinfo->tm_year + 1900 << "-" << timeinfo->tm_yday + 1 << " " << timeinfo->tm_hour << ":" << timeinfo->tm_min << ":" << timeinfo->tm_sec;
    return ssDate.str();
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
bool SoundVelocityProfile::readSectionHeader(std::string & row, uint64_t &nbrM,double &lat, double &lon)
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
  * Returns the latitude or the longitude in format dd:mm:ss
  *
  * @param value the latitude or longitude to convert
  * @param direction direction(N,W,E,S) of the latitude or longitude
  */
  std::string SoundVelocityProfile::latlongFormat(double value, std::string direction)
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


#endif

