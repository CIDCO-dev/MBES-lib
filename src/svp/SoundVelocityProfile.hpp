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
    SoundVelocityProfile() {
        longitude = latitude = nan("");
        timestamp = 0;
    }

    /**Destroys a sound velocity*/
    ~SoundVelocityProfile() {

    }

    /**Returns the size of the SoundVelocityProfile*/
    unsigned int getSize() {
        return getDepths().size();
    };

    /**Returns the latitude of the SoundVelocityProfile*/
    double getLatitude() {
        return latitude;
    }

    /**
     * Sets the latitude of the SoundVelocityProfile
     *
     * @param l the new latitude
     */
    void setLatitude(double l) {
        latitude = l;
    }

    /**Returns the longitude of the SoundVelocityProfile*/
    double getLongitude() {
        return longitude;
    }

    /**
     * Sets the longitude of the SoundVelocityProfile
     *
     * @param l the new longitude of the SoundVelocityProfile
     */
    void setLongitude(double l) {
        longitude = l;
    }

    /**Return the timestamp of the sound velocity*/
    uint64_t getTimestamp() {
        return timestamp;
    };

    /**
     * Sets the timestamp of the SoundVelocityProfile
     *
     * @param t the new timestamp
     */
    void setTimestamp(uint64_t t) {
        timestamp = t;
    };

    /**
     * Adds a new value in the vector depths and speeds
     *
     * @param depth value to add in depths
     * @param soundSpeed value to add in speeds
     */
    void add(double depth, double soundSpeed) {
        samples.push_back(std::make_pair(depth, soundSpeed));
    }

    /**Returns the depths vector*/
    Eigen::VectorXd & getDepths() {
        //lazy load internal vector
        if ((unsigned int) depths.size() != samples.size()) {
            depths.resize(samples.size());

            for (unsigned int i = 0; i < samples.size(); i++) {
                depths(i) = samples[i].first;
            }
        }

        return depths;
    }

    /**Returns the speeds vector*/
    Eigen::VectorXd & getSpeeds() {
        //lazy load internal vector
        if ((unsigned int) speeds.size() != samples.size()) {
            speeds.resize(samples.size());

            for (unsigned int i = 0; i < samples.size(); i++) {
                speeds(i) = samples[i].second;
            }
        }

        return speeds;
    }
    
    /**Returns the sound speed gradient*/
    Eigen::VectorXd & getSoundSpeedGradient() {
        if( (unsigned int) gradient.size() != samples.size()-1) {
            gradient.resize(samples.size() - 1);
            
            for (unsigned int k=0; k < samples.size()-1; k++){
                gradient(k) = (getSpeeds()(k+1)- getSpeeds()(k))/(getDepths()(k+1)- getDepths()(k));
            }
        }
        
        return gradient;
    }
    
    /**Returns the layer index at specified depth*/
    unsigned int getLayerIndexForDepth(double depth) {
        // Warning: do not confuse layer with svp sample
        // a layer is the space above or beneath a sample
        
        if(depth < samples[0].first) {
            return 0; // the 0-th layer is shallower than the shallowest svp sample
        }
        
        for (unsigned int k=0; k < samples.size()-1; k++){
            if( depth >= samples[k].first && depth < samples[k+1].first) {
                return k+1;
            }
        }
        
        // This layer is deeper than the deepest svp sample
        return samples.size();
    }

    /**
     * Returns the stream in which this ping will be written
     *
     * @param os the stream in which to write this ping
     * @param obj the ping to write in the stream
     */
    friend std::ostream& operator<<(std::ostream & os, const SoundVelocityProfile & obj) {
        return os <<
                "timestamp: " << obj.timestamp << std::endl <<
                "latitude: " << obj.latitude << std::endl <<
                "longitude: " << obj.longitude << std::endl;
    }

private:

    /**timestamp value of the SoundVelocityProfile (micro-second)*/
    uint64_t timestamp; //timestamp

    /**latitude value of the SoundVelocityProfile*/
    double latitude;

    /**longitude value of the SoundVelocityProfile*/
    double longitude;

    /**vector that contains the dephts of the SoundVelocityProfile*/
    Eigen::VectorXd depths;

    /**vector that contain the speeds of the SoundVelocityProfile*/
    Eigen::VectorXd speeds;
    
    /**vector that contain sound speed gradient*/
    Eigen::VectorXd gradient;

    /**vector that contain the depths and the speeds*/
    std::vector<std::pair<double, double>> samples;
};

#endif /* SOUNDVELOCITYPROFILE_HPP */
