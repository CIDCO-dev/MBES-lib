/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/* 
 * File:   SidescanPing.hpp
 * Author: glm, jordan
 *
 * Created on August 28, 2019, 5:34 PM
 */

#ifndef SIDESCANPING_HPP
#define SIDESCANPING_HPP

#include <vector>
#include <cstdint>
#include "../Position.hpp"
#include "../Attitude.hpp"


class SidescanPing {
public:
    SidescanPing();
    SidescanPing(const SidescanPing& orig);
    ~SidescanPing();
    
    void setDistancePerSample(double d){distancePerSample = d;};
    double getDistancePerSample(){ return distancePerSample;};
    
    std::vector<double> & getRawSamples(){ return rawSamples;};
    
    std::vector<double> & getSamples(){ return samples;};
    
    void setSamples(std::vector<double> & s){
        samples = s;
    }

    void setRawSamples(std::vector<double> & s){
        rawSamples = s;
    }
    
    int getChannelNumber(){ return channelNumber;};
    void setChannelNumber(int channel){ channelNumber = channel;};
    
    uint64_t getTimestamp() {return timestamp;};
    void setTimestamp(uint64_t newTimestamp){ timestamp=newTimestamp;};
    
    Attitude * getAttitude(){ return attitude;};
    void       setAttitude(Attitude * newAttitude){attitude=newAttitude;};
    
    Position * getPosition(){ return position;};
    void       setPosition(Position * newPosition){position=newPosition;};
    
    double getLayback(){return layback;}
    void setLayback(double layback){this->layback = layback;}

    double getSensorDepth(){return sensorDepth;}
    void setSensorDepth(double sensorDepth){this->sensorDepth = sensorDepth;}
    
    double getSoundVelocity(){ return soundVelocity;}
    void   setSoundVelocity(double sv){ this->soundVelocity = sv;}

    double getSensorPrimaryAltitude(){return sensorPrimaryAltitude;}
    void setSensorPrimaryAltitude(double sensorPrimaryAltitude){this->sensorPrimaryAltitude = sensorPrimaryAltitude;}

    double getSlantRange(){return slantRange;}
    void setSlantRange(double slantRange){this->slantRange = slantRange;}

    double getTimeDuration(){return timeDuration;}
    void setTimeDuration(double timeDuration){this->timeDuration = timeDuration;}

private:
    std::vector<double> samples; //we will boil down all the types to double. This is not a pretty hack, but we need to support every sample type
    std::vector<double> rawSamples;
    double      soundVelocity;
    double      distancePerSample;
    int         channelNumber;
    uint64_t    timestamp;
    Attitude *  attitude;
    Position *  position;
    double      layback;
    double      sensorDepth;
    double      sensorPrimaryAltitude;
    double      slantRange;
    double      timeDuration;
};

#endif /* SIDESCANPING_HPP */

