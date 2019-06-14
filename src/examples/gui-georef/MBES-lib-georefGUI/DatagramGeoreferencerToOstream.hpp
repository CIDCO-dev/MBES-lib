/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef DATAGRAMGEOREFERENCERFORGUI_HPP
#define DATAGRAMGEOREFERENCERFORGUI_HPP

#include <Eigen/Dense>

#include <iostream> // Temp while using cout?
#include <iomanip> // Temp while using cout?

#include "../../../georeferencing/DatagramGeoreferencer.hpp"
#include "../../../Ping.hpp"
#include "../../../Position.hpp"
#include "../../../Attitude.hpp"
#include "../../../georeferencing/Georeferencing.hpp"
#include "../../../svp/SoundVelocityProfile.hpp"
#include "../../../svp/SoundVelocityProfileFactory.hpp"
#include "../../../datagrams/DatagramEventHandler.hpp"
#include "../../../math/Interpolation.hpp"

/*!
* \brief "Datagram Georeferencer to an ostream" class, extention of the Datagram Georeferencer class
* \author Christian Bouchard
*/
class DatagramGeoreferencerToOstream : public DatagramGeoreferencer{
public:
  /**Creates a datagram georeferencer*/
  DatagramGeoreferencerToOstream( std::ostream & out )
  : out( out )
  {
    out << std::setprecision(6);
    out << std::fixed;
  }

  /**Destroys the datagram georeferencer*/
  virtual ~DatagramGeoreferencerToOstream() {}

  /**
  * Displays a georeferenced ping's info
  *
  * @param georeferencedPing
  * @param quality the quality flag
  * @param intensity the intensity flag
  */
  virtual void processGeoreferencedPing(Eigen::Vector3d & georeferencedPing,uint32_t quality,int32_t intensity);

private:
  std::ostream & out; // ostream: can be used for a file or for std::cout


};

void DatagramGeoreferencerToOstream::processGeoreferencedPing(Eigen::Vector3d & georeferencedPing,uint32_t quality,int32_t intensity){
  out << georeferencedPing(0) << " " << georeferencedPing(1) << " " << georeferencedPing(2)
  << " " << quality  << " " << intensity << std::endl;
}

#endif
