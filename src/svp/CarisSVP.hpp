/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   CarisSVP.hpp
 * Author: jordan
 */

#ifndef CARISSVP_HPP
#define CARISSVP_HPP

#include <vector>
#include <string>
#include <time.h>

#include "SoundVelocityProfile.hpp"

class CarisSVP {
private:

    /**Vector of SoundVelocityProfile*/
    std::vector<SoundVelocityProfile*> svps;

    void clearSVPs() {
        for (unsigned int i = 0; i < svps.size(); ++i) {
            delete svps[i];
        }

        svps.clear();
    }

    bool readSectionHeader(std::string & line, uint64_t & timestamp, double & latitude, double & longitude) {

        int latdegrees;
        int latminute;
        double latsecond;

        int londegrees;
        int lonminute;
        double lonsecond;

        int year;
        int yday;
        int hour;
        int minute;
        int second;

        if (std::sscanf(line.c_str(), "Section %d-%d %d:%d:%d %d:%d:%lf %d:%d:%lf", &year, &yday, &hour, &minute, &second, &latdegrees, &latminute, &latsecond, &londegrees, &lonminute, &lonsecond) == 11) {
            latitude = latdegrees + latminute * 60 + latsecond * 60 * 60;
            longitude = londegrees + lonminute * 60 + lonsecond * 60 * 60;

            std::string carisSvpTime =
                    std::to_string(year) + "-" +
                    std::to_string(yday) + " " +
                    std::to_string(hour) + ":" +
                    std::to_string(minute) + ":" +
                    std::to_string(second);

            timestamp = TimeUtils::convertCarisSvpDate2EpochMicro(carisSvpTime.c_str());

            return true;
        }

        return false;
    }

public:

    CarisSVP() {

    }

    ~CarisSVP() {
        clearSVPs();
    }

    std::vector<SoundVelocityProfile*> & getSvps() {
        return svps;
    }
    
    void setSvps(std::vector<SoundVelocityProfile*> & svps) {
        this->svps = svps;
    }

    bool writeSvpFile(std::string & filename) {
        std::ofstream out;
        out.open(filename);
        if (!out.is_open()) {
            std::cerr << "Could not write svp to file: " << filename << std::endl;
            return false;
        } else {
            //write VERSION
            out << "[SVP_VERSION_2]" << std::endl;
            
            //write filename
            out << filename << std::endl;
            
            //write sections
            for(unsigned int i=0; i < svps.size(); ++i) {
                SoundVelocityProfile * svp = svps[i];
                
                int latDegrees = std::trunc(svp->getLatitude());
                int latMinutes = std::trunc((svp->getLatitude()-latDegrees)*60);
                double latSeconds = ((svp->getLatitude()-latDegrees)*60 - latMinutes)*60;
                
                std::stringstream coordStream;
                coordStream << latDegrees << ":" << latMinutes << ":" << latSeconds;
                
                int lonDegrees = std::trunc(svp->getLongitude());
                int lonMinutes = std::trunc((svp->getLongitude()-lonDegrees)*60);
                double lonSeconds = ((svp->getLongitude()-lonDegrees)*60 - lonMinutes)*60;
                coordStream << " " << lonDegrees << ":" << lonMinutes << ":" << lonSeconds;
                
                std::string carisDateFormat = TimeUtils::julianTime(svp->getTimestamp());
                
                out << "Section " << carisDateFormat << " " << coordStream.str() << std::endl;
                
                for(unsigned int j=0; j<svp->getDepths().rows(); ++j) {
                    out << svp->getDepths()[j] << " " << svp->getSpeeds()[j] << std::endl;
                }
            }
        }
        
        return true;
    }

    bool readSvpFile(std::string & filename) {

        clearSVPs();

        std::ifstream inFile(filename);

        if (!inFile) {
            std::cerr << "Cannot read svp file: " << filename << std::endl;
            return false;
        } else {
            //Read VERSION String
            std::string version;
            std::getline(inFile, version);

            int ver;
            if (std::sscanf(version.c_str(), "[SVP_VERSION_%d]", &ver) != 1 && ver != 2) {
                std::cerr << "CARIS SVP VERSION: " << version << std::endl;
                std::cerr << "Currently only VERSION 2 is supported" << std::endl;
                inFile.close();
                return false;
            }

            //Read filename
            std::string fname;
            std::getline(inFile, fname);

            //Start reading SVP sections
            SoundVelocityProfile * currentSVP = NULL;
            std::string line;
            while (std::getline(inFile, line)) {

                if (line.rfind("Section", 0) == 0) {
                    //we are at a new SVP profile
                    if (currentSVP == NULL) {
                        currentSVP = new SoundVelocityProfile();
                    } else {
                        svps.push_back(currentSVP);
                        currentSVP = new SoundVelocityProfile();
                    }

                    uint64_t currentSvpTimestamp;
                    double currentSvpLatitude;
                    double currentSvpLongitude;

                    if (readSectionHeader(line, currentSvpTimestamp, currentSvpLatitude, currentSvpLongitude)) {
                        currentSVP->setTimestamp(currentSvpTimestamp);
                        currentSVP->setLatitude(currentSvpLatitude);
                        currentSVP->setLongitude(currentSvpLongitude);
                    } else {
                        std::cerr << "Can't parse time and location of SVP." << std::endl;
                        std::cerr << line << std::endl;
                        inFile.close();
                        return false;
                    }
                } else {
                    double depth;
                    double speed;
                    if (std::sscanf(line.c_str(), "%lf %lf", &depth, &speed) == 2) {
                        currentSVP->add(depth, speed);
                    } else {
                        //ignore bad lines
                    }
                }
            }

            if (currentSVP != NULL) {
                svps.push_back(currentSVP);
            } else {
                std::cerr << "Couldn't read SVP from file: " << filename << std::endl;
                return false;
            }
        }

        return true;
    }
};

#endif /* CARISSVP_HPP */

