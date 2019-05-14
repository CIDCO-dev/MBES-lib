/*
* Copyright 2018 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef SWATH_HPP
#define SWATH_HPP

#include <iostream>
#include <vector>
#include "Ping.hpp"

/*!
* \brief Swath class
* \author Guillaume Labbe-Morissette, Jordan McManus, Emile Gagne
* \date September 14, 2018, 2:39 PM
*/
class Swath {
private:

  /**Vector that contains the pings of the swath*/
  std::vector<Ping *>* pings = NULL;

public:

  /**
  * Creates a swath
  *
  * @param pings the pings that should be present in the swath
  */
  Swath(std::vector<Ping*>* pings) : pings(pings) {
  }

  /**
  * Destroys the swath and its pings
  */
  ~Swath() {
    if (pings != NULL) {
      for (unsigned int i = 0; i < pings->size(); i++) {
        delete (*pings)[i];
      }

      delete pings;
    }
  }

  /**
  * Returns the vector pings
  */
  std::vector<Ping*>* getPings() {
    return pings;
  }

  /**
  *
  */
  friend std::ostream& operator<<(std::ostream& os, const Swath& obj) {
    for (unsigned int i = 0; i < (*obj.pings).size(); i++) {
      os << *((*obj.pings)[i]) << std::endl;
    }
    return os;
  }

};

#endif /* SWATH_HPP */
