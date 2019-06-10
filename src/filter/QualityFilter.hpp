/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef QUALITYFILTER_HPP
#define QUALITYFILTER_HPP

#include "PointFilter.hpp"

/*!
* \brief Quality filter class.
* \author Guillaume Labbe-Morissette
*
* Extends from the Point filter class
*/
class QualityFilter : public PointFilter{
public:

  /**
  * Creates a quality filter
  *
  * @param minimumQuality the minimal quality accepted
  */
  QualityFilter(int minimumQuality) : minimumQuality(minimumQuality){

  }

  /**Destroys the quality filter*/
  ~QualityFilter(){

  }

  /**
  * Returns true if the quality received is lower than the minimum accepted
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

  /**Minimal quality accepted*/
  unsigned int minimumQuality;

};

#endif
