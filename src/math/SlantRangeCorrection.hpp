/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/* 
 * @author Guillaume Labbé-Morissette
 */

#ifndef SLANTRANGECORRECTION_HPP
#define SLANTRANGECORRECTION_HPP

#include <vector>


class SlantRangeCorrection{
public:
    /**
     * 
     * @param samples the samples, sampled along the slant
     * @param slantRange distance along the slant
     * @param roll roll, taken as positive = starboard
     * @param beamAngle taken from the nadir
     */
    void correct(std::vector<double> & samples,double slantRange,double roll,double beamAngle,std::vector<double> & out){
        double theta = beamAngle - roll;
        double sT = sin(theta);
        
        //reproject pixels into bins according to the slant range
        out.resize(ceil(slantRange * sT));
        
        for(unsigned int i=0;i<samples.size();i++){
            double pixelIndex = (samples.size() * sT * (slantRange - sqrt(slantRange - ( ((double) i*slantRange) / (double)samples.size())))) / (double) slantRange; 
            
            out[floor(pixelIndex)]=samples[i];
        }
    }
    
};

#endif /* SLANTRANGECORRECTION_HPP */

