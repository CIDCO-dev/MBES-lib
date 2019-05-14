/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef SOUNDVELOCITYPROFILEFACTORY_HPP
#define SOUNDVELOCITYPROFILEFACTORY_HPP

/*!
* \brief SoundVelocityProfile factory class
* \author ?
*/
class SoundVelocityProfileFactory{
public:
	/**
	* Returns a SoundVelocityProfile model with salt water
	*/
	static SoundVelocityProfile * buildSaltWaterModel(){
		SoundVelocityProfile * svp = new SoundVelocityProfile();
		//TODO: set time/location?
		svp->add(0,1520);
		svp->add(15000,1520);
		return svp;
	}
};


#endif
