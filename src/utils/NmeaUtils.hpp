#ifndef NMEAUTILS_HPP
#define NMEAUTILS_HPP

#include <limits>
#include <string>
#include <sstream>
#include <iostream>
#include <cstdlib>

class NmeaUtils{
public:
    static double extractHeightFromGGK(std::string & ggkString);
    static double extractHeightFromGGA(std::string & ggaString);
};

/**
 * @brief extractHeightFromGGK Extracts ellipsoidal height from GGK
 * @param ggkString
 * @return
 */
double NmeaUtils::extractHeightFromGGK(std::string & ggkString){
    std::stringstream ss(ggkString);
    std::string token;

    while(std::getline(ss,token,',')){
        if(token.find("EHT")!= std::string::npos){
            return atof(token.substr(3).c_str());
        }
    }

    return std::numeric_limits<double>::quiet_NaN();
}

/**
 * @brief extractHeightFromGGA Extracts ellipsoidal height from GGA
 * @param ggaString
 * @return
 */
double NmeaUtils::extractHeightFromGGA(std::string & ggaString){
    std::stringstream ss(ggaString);
    std::string token;

    double orthometricHeight = std::numeric_limits<double>::quiet_NaN();
    double geoidSeperation   = std::numeric_limits<double>::quiet_NaN();

    int i=0;

    while(std::getline(ss,token,',')){
        if(i==9){
            //orthometric height
            orthometricHeight = atof(token.c_str());
        }
        else if(i==11){
            //geoid seperation
            geoidSeperation = atof(token.c_str());
        }

        if(!std::isnan(orthometricHeight) && !std::isnan(geoidSeperation)){
            return orthometricHeight + geoidSeperation;
        }

        i++;
    }

    return std::numeric_limits<double>::quiet_NaN();
}

#endif
