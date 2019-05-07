#ifndef POINTFILTER_HPP
#define POINTFILTER_HPP

/**
 * @author Glm
 */

/*!
 * \brief Point filter class
 */
class PointFilter{
   public:

       /**Create a point filter*/
        PointFilter(){

        }

        /**Destroy the point filter*/
        ~PointFilter(){

        }

        /**
         * return true if we remove this point
         * 
         * @param microEpoch timestamp of the point
         * @param x x position of the point
         * @param y y position of the point
         * @param z z position of the point
         * @param quality quality of the point
         * @param intensity intensity of the point
         */
        virtual bool filterPoint(uint64_t microEpoch,double x,double y,double z, uint32_t quality,uint32_t intensity) = 0;
};

#endif
