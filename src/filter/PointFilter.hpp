/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef POINTFILTER_HPP
#define POINTFILTER_HPP

/*!
* \brief Point filter class
* \author Guillaume Labbe-Morissette
*/
class PointFilter{
public:

  /**Creates a point filter*/
  PointFilter(){

  }

  /**Destroys the point filter*/
  ~PointFilter(){

  }

  /**
  * Returns true if we removed this point
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
