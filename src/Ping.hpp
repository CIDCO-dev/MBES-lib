/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   Ping.hpp
 * Author: glm,jordan, emilegagne
 *
 * Created on September 14, 2018, 10:10 AM
 */

#ifndef PING_HPP
#define PING_HPP

#include <iostream>
#include <Eigen/Dense>
#include "Attitude.hpp"
#include "Position.hpp"

/*!
 * \brief Classe d'un ping contenant la vitesse du son, le temps de déplacement, l'angle qui passe au long et l'angle qui passe à travers
 */
class Ping {
private:
    
    /**Valeur de temps calculé depuis janvier 1970 (micro-seconde)*/
    uint64_t timestamp; //in microseconds since epoch
    
    /**Valeur identifiant le ping*/
    uint64_t id;
    
    /**Valeur de la qualité du ping*/
    uint32_t quality;
    
    /**Valeur de l'intensité du ping*/
    uint32_t intensity;

    /**Valeur de la vitesse du son d'une surface*/
    double surfaceSoundSpeed;
    
    /**Valeur du temps de transition entre les deux points*/
    double twoWayTravelTime;
    
    /**Valeur de l'angle qui passe au long*/
    double alongTrackAngle;  // In degrees, AKA emission angle, alpha, kappa or tilt angle
    
    /**Valeur de l'angle qui passe à travers*/
    double acrossTrackAngle; // In degrees, AKA reception angle, beta, zeta, beam angle


    /*Trigonometry is stored to prevent redundant recalculations*/
    /**Valeur sinus de l'angle qui passe au long*/
    double sA;
    
    /**Valeur cosinus de l'angle qui passe au long*/
    double cA;

    /**Valeur sinus de l'angle qui passe à travers*/
    double sB;
    
    /**Valeur cosinus de l'angle qui passe à travers*/
    double cB;

public:
    
    /**
     * Crée le ping
     * 
     * @param microEpoch valeur temps du ping
     * @param id identifiant du ping
     * @param quality qualité du ping
     * @param intensity intensité du ping
     * @param surfaceSoundSpeed vitesse du son de la surface du ping
     * @param twoWayTravelTime durée de la transition entre les deux points 
     * @param alongTrackAngle angle qui passe au long
     * @param acrossTrackAngle angle qui passe à travers
     */
    Ping(
	uint64_t microEpoch,
	long     id,
	uint32_t quality,
	uint32_t intensity,

        double surfaceSoundSpeed,
        double twoWayTravelTime,
        double alongTrackAngle,
        double acrossTrackAngle
    ):
    timestamp(microEpoch),
    id(id),
    quality(quality),
    intensity(intensity),
    surfaceSoundSpeed(surfaceSoundSpeed),
    twoWayTravelTime(twoWayTravelTime),
    alongTrackAngle(alongTrackAngle),
    acrossTrackAngle(acrossTrackAngle),
    sA(sin(alongTrackAngle*D2R)),
    cA(cos(alongTrackAngle*D2R)),
    sB(sin(acrossTrackAngle*D2R)),
    cB(cos(acrossTrackAngle*D2R)){
    }

    /** Détruit le ping*/
    ~Ping() {

    }

    /**Retourne l'angle passant à travers*/
    double getAcrossTrackAngle() {
        return acrossTrackAngle;
    }

    /**Retourne l'angle passant au long*/
    double getAlongTrackAngle() {
        return alongTrackAngle;
    }

    /**Retourne le cosinus de l'angle passant au long*/
    double getCA() {
        return cA;
    }

    /**Retourne le cosinus de l'angle passant à travers*/
    double getCB() {
        return cB;
    }

    /**Retourne la valeur temps du ping*/
    double getTimestamp() {
        return timestamp;
    }

    /**Retourne le sinus de l'angle passant au long*/
    double getSA(){
	return sA;
    }

    /**Retourne le sinus de l'angle passant à travers*/
    double getSB() {
        return sB;
    }

    /**Retourne la vitesse du son d'une surface*/
    double getSurfaceSoundSpeed(){
	return surfaceSoundSpeed;
    }

    /**Retourne le temps de transition entre deux points*/
    double getTwoWayTravelTime() {
        return twoWayTravelTime;
    }

    /**Retourne la qualité du ping*/
    uint32_t getQuality() { return quality;}

    /**Retourne l'intensité du ping*/
    uint32_t getIntensity() { return intensity;}

};


#endif /* PING_HPP */

