/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   SoundVelocityProfile.h
 * Author: glm,jordan, emilegagne
 *
 * Created on August 22, 2018, 10:30 AM
 */

#ifndef SOUNDVELOCITYPROFILE_HPP
#define SOUNDVELOCITYPROFILE_HPP

#include <list>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>

/*!
 * \brief Classe de Vélocité de son
 */
class SoundVelocityProfile {
public:

    /**Crée une vélocité*/
    SoundVelocityProfile();

    /**Détruit la vélocité*/
    ~SoundVelocityProfile();


    /**
     * Écrit les informations de la vélocité dans un fichier
     * 
     * @param filename nom du fichier à écrire
     */
    void write(std::string & filename);

    /**Retourne la valeur size*/
    unsigned int getSize() {
        return size;
    };

    /**Retourne la latitude de la vélocité*/
    double getLatitude() 	 { return latitude; }
    
    /**
     * Change la latitude de la vélocité
     * 
     * @param l nouvelle latitude
     */
    void   setLatitude(double l) { latitude=l;}

    /**Retourne la longitude de la vélocité*/
    double getLongitude()	 { return longitude;}
    
    /**
     * Change la longitude de la véloncité
     * 
     * @param l nouvelle longitude
     */
    void   setLongitude(double l){ longitude=l;}

    /**Retourne la valeur temps de la vélocité*/
    uint64_t getTimestamp(){ return microEpoch;};
    
    /**
     * Change la valeur temps de la vélocité
     * 
     * @param t nouvelle valeur temps
     */
    void     setTimestamp(uint64_t t) { microEpoch=t;};
    
    /**
     * Ajoute ajoute une valeur dans le vecteur depths et speeds
     * 
     * @param depth valeur à ajouter dans le vecteur depths
     * @param soundSpeed valeur à ajouter dans le vecteur speeds
     */
    void     add(double depth,double soundSpeed);

    /**Retourne le vecteur depths*/
    Eigen::VectorXd & getDepths();
    
    /**Retourne le vecteur speeds*/
    Eigen::VectorXd & getSpeeds();

private:
    
    /**valeur taille de la vélocité*/
    unsigned int size;

    /**valeur temps calculé depuis janvier 1970 (micro-seconde)*/
    uint64_t  microEpoch; //timestamp

    /**valeur de la latitude de la vélocité*/
    double latitude;
    
    /**valeur de la longitude de la vélocité*/
    double longitude;

    /**vecteurs contenant les depth*/
    Eigen::VectorXd depths;
    
    /**vecteurs contenant les speed*/
    Eigen::VectorXd speeds;

    std::vector<std::pair<double,double>> samples;
};

/**Crée une vélocité*/
SoundVelocityProfile::SoundVelocityProfile() {
    longitude = latitude = nan("");
};

/**Détruit la vélocité*/
SoundVelocityProfile::~SoundVelocityProfile() {
};

/**
     * Ajoute ajoute une valeur dans le vecteur depths et speeds
     * 
     * @param depth valeur à ajouter dans le vecteur depths
     * @param soundSpeed valeur à ajouter dans le vecteur speeds
     */
void SoundVelocityProfile::add(double depth,double soundSpeed){
	samples.push_back(std::make_pair(depth,soundSpeed));
}

/**
     * Écrit les informations de la vélocité dans un fichier
     * 
     * @param filename nom du fichier à écrire
     */
void SoundVelocityProfile::write(std::string & filename){
	std::ofstream out(filename);

	if(out.is_open()){
		//TODO: write proper date and lat/lon
		out << "[SVP_VERSION_2]" << "\r\n";
		out << filename << "\r\n";
		out << "Section 2000-307 18:59:00 43:04:40 -070:42:42 This time and coordinates are wrong" << "\r\n"; //FIXME: put date as yyyy-ddd hh:mm:ss dd:mm:ss (lat) dd:mm:ss (lon)

		for(auto i=samples.begin();i!=samples.end();i++){
			out << (*i).first << " " << (*i).second << "\r\n";
		}

		out.close();
	}
}

/**Retourne le vecteur depths*/
Eigen::VectorXd & SoundVelocityProfile::getDepths(){
	//lazy load internal vector
	if((unsigned int)depths.size() != samples.size()){
		depths.resize(samples.size());

		for(unsigned int i = 0;i<samples.size();i++){
			depths(i)=samples[i].first;
		}
	}

	return depths;
}

/**Retourne le vecteur speeds*/
Eigen::VectorXd & SoundVelocityProfile::getSpeeds(){
	//lazy load internal vector
	if((unsigned int)speeds.size() != samples.size()){
		speeds.resize(samples.size());

		for(unsigned int i=0;i<samples.size();i++){
			speeds(i)=samples[i].second;
		}
	}

	return speeds;
}

#endif /* SOUNDVELOCITYPROFILE_HPP */
