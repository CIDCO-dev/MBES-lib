/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef NMEAUTILS_HPP
#define NMEAUTILS_HPP

#include <limits>
#include <string>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <cmath>

/*!
* \brief NmeaUtils class
* \author Guillaume Labbe-Morissette, Jordan McManus, Emile Gagne
*/
class NmeaUtils{
public:
  static double extractHeightFromGGK(std::string & ggkString);
  static double extractHeightFromGGA(std::string & ggaString);
};

#endif
