/*
 * Copyright 2018 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   Swath.hpp
 * Author: jordan,glm,emilegagne
 *
 * Created on September 14, 2018, 2:39 PM
 */

#ifndef SWATH_HPP
#define SWATH_HPP

#include <iostream>
#include <vector>
#include "Ping.hpp"

/*!
 * \brief Swath class
 */
class Swath {
private:
    
    /**Vector who contains the pings of the swath*/
    std::vector<Ping *>* pings = NULL;

public:

    /**
     * Create a swath
     * 
     * @param pings the pings who should be present in the swath
     */
    Swath(std::vector<Ping*>* pings) : pings(pings) {
    }

    /**
     * Destroy the swath and his pings
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
     * Return the vector pings
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

