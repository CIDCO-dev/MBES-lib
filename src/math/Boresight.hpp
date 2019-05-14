/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef BORESIGHT_HPP
#define BORESIGHT_HPP

/*!
* \brief Boresight util. Give matrix between IMU and Sonar.
* \author ?
*/
class Boresight{
public:
	/**
	* Builds and outputs the boresight matrix.
	*
	* @param M the boresight matrix
	* @param boresight the boresight angles
	*/
	static void buildMatrix(Eigen::Matrix3d & M,Attitude & boresight){
		double ch = boresight.getCh();
		double sh = boresight.getSh();
		double cp = boresight.getCp();
		double sp = boresight.getSp();
		double cr = boresight.getCr();
		double sr = boresight.getSr();

		M << 	ch*cp , ch*sp*sr - sh*cr  , ch*sp*cr + sh*sr,
		sh*cp , sh*sp*sr + ch*cr  , sh*sp*cr - ch*sr,
		-sp   , cp*sr		  , cp*cr;
	}

};

#endif
