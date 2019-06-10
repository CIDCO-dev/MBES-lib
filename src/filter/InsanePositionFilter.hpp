/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef INSANEPOSITIONFILTER_HPP
#define INSANEPOSITIONFILTER_HPP

#include "PointFilter.hpp"

/*!
* \brief Insane position filter class.
* \author Emile Gagne
*
* Extends from the PointFilter class
*/
class InsanePositionFilter : public PointFilter{
public:

  /**
  * Creates an insane position filter
  */
  InsanePositionFilter(){

  }

  /**Destroys the insane position filter*/
  ~InsanePositionFilter(){

  }

  /**
  * Returns true if the position received seems insane
  *
  * @param microEpoch timestamp of the point
  * @param x x position of the point
  * @param y y position of the point
  * @param z z position of the point
  * @param quality quality of the point
  * @param intensity intensity of the point
  */
  bool filterPoint(uint64_t microEpoch,double x,double y,double z, uint32_t quality,uint32_t intensity){
    bool insaneX = ((x>1.00*100000000)||(x<-1.00*100000000));
    bool insaneY = ((y>1.00*100000000)||(y<-1.00*100000000));
    bool insaneZ = ((z>1.00*100000000)||(z<-1.00*100000000));
    return (insaneX||insaneY||insaneZ);
  }
};

#endif
