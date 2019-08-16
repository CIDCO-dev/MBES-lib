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
    static void correct(std::vector<double> & samples,double slantRange,double roll,double beamAngle,std::vector<double> & out){
        double theta = beamAngle - roll;
        double sT = sin(theta*(M_PI/180));
        
        double groundRangePixels =  ceil(std::abs(samples.size() * sT)); //in pixels
        
        //reproject pixels into bins according to the slant range
        //std::cerr << "Theta: " << theta << std::endl;
        //std::cerr << "Slant: " <<  samples.size() << " pixels     Ground: " << groundRangePixels << "pixels" << std::endl;
        
        out.resize(groundRangePixels);
        
        for(unsigned int i=0;i<samples.size();i++){
            int pixelIndex = floor(std::abs(sT * i)); 

            //std::cerr << "Index: " << i << "       New index: " << pixelIndex << std::endl;
            out[pixelIndex]=samples[i];
        }
    }
    
};

#endif /* SLANTRANGECORRECTION_HPP */

