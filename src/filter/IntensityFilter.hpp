#ifndef INTENSITYFILTER_HPP
#define INTENSITYFILTER_HPP

/**
 * @author EmileGagne
 */

#include "PointFilter.hpp"

/*!
 * \brief Intensity filter class extend of the Point filter class
 */
class IntensityFilter : public PointFilter{
   public:

       /**
        * Create a intensity filter
        * 
        * @param minimumIntensity the minimal intensity accepted
        */
        IntensityFilter(int minimumIntensity) : minimumIntensity(minimumIntensity){

        }

        /**Destroy the intensity filter*/
        ~IntensityFilter(){

        }

        /**
         * return true if the intensity receive is low then the minimum accepted
         * 
         * @param microEpoch timestamp of the point
         * @param x x position of the point
         * @param y y position of the point
         * @param z z position of the point
         * @param quality quality of the point
         * @param intensity intensity of the point
         */
        bool filterPoint(uint64_t microEpoch,double x,double y,double z, uint32_t quality,uint32_t intensity){
                return intensity < minimumIntensity;
        }

  private:

      /**minimal intensity accepted*/
        unsigned int minimumIntensity;

};

#endif
