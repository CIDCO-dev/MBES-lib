#ifndef INTENSITYFILTER_HPP
#define INTENSITYFILTER_HPP

#include "PointFilter.hpp"

/*!
* \brief Intensity filter class.
* \author ?
*
* Extends from the Point filter class
* \author Emile Gagne
*/
class IntensityFilter : public PointFilter{
public:

  /**
  * Creates an intensity filter
  *
  * @param minimumIntensity the minimal intensity accepted
  */
  IntensityFilter(int minimumIntensity) : minimumIntensity(minimumIntensity){

  }

  /**Destroys the intensity filter*/
  ~IntensityFilter(){

  }

  /**
  * Returns true if the intensity received is lower than the minimum accepted
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

  /**Minimal intensity accepted*/
  unsigned int minimumIntensity;

};

#endif
