#ifndef QUALITYFILTER_HPP
#define QUALITYFILTER_HPP

#include "PointFilter.hpp"

/*!
 * \brief Quality filter class extend of the Point filter class
 */
class QualityFilter : public PointFilter{
   public:

       /**
        * Create a quality filter
        * 
        * @param minimumQuality the minimal quality accepted
        */
        QualityFilter(int minimumQuality) : minimumQuality(minimumQuality){

        }

        /**Destroy the quality filter*/
        ~QualityFilter(){

        }

        /**
         * return true if the quality receive is low then the minimum accepted
         * 
         * @param microEpoch timestamp of the point
         * @param x x position of the point
         * @param y y position of the point
         * @param z z position of the point
         * @param quality quality of the point
         * @param intensity intensity of the point
         */
        bool filterPoint(uint64_t microEpoch,double x,double y,double z, uint32_t quality,uint32_t intensity){
                return quality < minimumQuality;
        }

  private:

      /**minimal quality accepted*/
        unsigned int minimumQuality;

};

#endif
