/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   Position.hpp
 * Author: glm,jordan,emilegagne
 *
 * Created on September 13, 2018, 3:29 PM
 */

#ifndef POSITION_HPP
#define POSITION_HPP

#include <iostream>
#include "utils/Constants.hpp"
#include <cmath>

/*!
 * \brief Classe d'une position
 */
class Position {
public:

    /**
     * Crée une position
     * 
     * @param microEpoch valeur de temps calculé depuis janvier 1970
     * @param latitude la latitude de la position
     * @param longitude la longitude de la position
     * @param ellipsoidalHeight l'hauteur de la position
     */
    Position(uint64_t microEpoch,double latitude, double longitude, double ellipsoidalHeight) :
    	timestamp(microEpoch),
	latitude(latitude),
    	longitude(longitude),
    	ellipsoidalHeight(ellipsoidalHeight),
    	slat(sin(latitude * D2R)),
    	clat(cos(latitude * D2R)),
    	slon(sin(longitude * D2R)),
    	clon(cos(longitude * D2R)) 
     {}

    /**Détruit la position*/
    ~Position() {
    }

    /**Retourne la valeur temps de la position*/
    uint64_t getTimestamp()		{ return timestamp; }
    
    /**
     * Change la valeur temps de la position
     * 
     * @param e nouvelle valeur temps
     */
    void     setTimestamp(uint64_t e)	{ timestamp = e;}

    /**Retourne la latitude de la position*/
    double   getLatitude()		{ return latitude; }
    
    /**
     * Change la latitude de la position
     * 
     * @param l nouvelle latitude
     */
    void     setLatitude(double l)	{ latitude = l; slat=sin(latitude * D2R); clat=cos(latitude * D2R);}

    /**Retourne la longitude de la position*/
    double   getLongitude()		{ return longitude; }
    
    /**
     * Change la longitude de la position
     * 
     * @param l nouvelle longitude
     */
    void     setLongitude(double l)     { longitude = l; slon=sin(longitude * D2R);clon=cos(longitude * D2R);}

    /**Retourne la hauteur de la position*/
    double   getEllipsoidalHeight()     	{ return ellipsoidalHeight; }
    
    /**
     * Change la hauteur de la position
     * 
     * @param h nouvelle hauteur
     */
    void     setEllipsoidalHeight(double h) 	{ ellipsoidalHeight = h;}

    /**Retourne la valeur sinus de la latitude*/
    double   getSlat()		{ return slat; }
    
    /**Retourne la valeur sinus de la longitude*/
    double   getSlon()		{ return slon; }
    
    /**Retourne la valeur cosinus de la latitude*/
    double   getClat()		{ return clat; }
    
    /**Retourne la valeur cosinus de la longitude*/
    double   getClon()		{ return clon; }

private:
    
    /**Valeur temps de la position*/
    uint64_t timestamp;

    // WGS84 position
    
    /**La latitude de la position*/
    double latitude;
    
    /**La longitude de la position*/
    double longitude;
    
    /**L'hauteur de la position*/
    double ellipsoidalHeight;

    /*Trigonometry is stored to prevent redundant recalculations*/
    
    /**Valeur sinus de la latitude*/
    double slat;
    
    /**Valeur cosinus de la latitude*/
    double clat;
    
    /**Valeur sinus de la longitude*/
    double slon;
    
    /**Valeur cosinus de la longitude*/
    double clon;

    /**
     * Crée une chaine de caractère sur les informations d'une position
     * 
     * @param os chaine de caractère où les informations doivent être placées
     * @param obj position où les informations doivent être obtenues
     */
    friend std::ostream& operator<<(std::ostream& os, const Position& obj) {
        return os << "Latitude: " << obj.latitude << std::endl << "Longitude: " << obj.longitude << std::endl << "Ellipsoidal Height: " << obj.ellipsoidalHeight << std::endl;
    }
};

#endif /* POSITION_HPP */

