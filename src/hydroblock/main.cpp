// C
#include <stdio.h>
#include <stdlib.h>

// c++
#include <string>

// MBES
#include "../math/Boresight.hpp"
#include "Hydroblock20Parser.hpp"
#include "../svp/CarisSvpFile.hpp"

#include <Eigen/Dense>
/*
Copyright 2022 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/*
 author Patrick Charron-Morneau
 hydroblock 2.0 parser
*/


void printUsage(){
	std::cerr << "\n\
	NAME\n\n\
	hydroblock-reader - output en format cidco (ASCII)\n\n\
	SYNOPSIS\n \
	hydroblock-reader gps.txt imu.txt sonar.txt\n\n\
	DESCRIPTION\n\n \
	Copyright 2022 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés" << std::endl;
	exit(1);
}

int main (int argc , char ** argv ){

	
	if(argc != 4){
		printUsage();
	}
	
	CarisSvpFile svps;
	std::string svpFilename = "../../test/data/SVP/SVP.txt";
	svps.readSvpFile(svpFilename);
	
	
	std::string gnssFilename = argv[1];
	std::string imuFilename = argv[2];
	std::string sonarFilename = argv[3];

	SvpSelectionStrategy * svpStrategy = new SvpNearestByTime();
	Georeferencing * georef = new GeoreferencingTRF();
	
	DatagramGeoreferencer  printer(*georef, *svpStrategy);
	printer.processSwathStart(1500.0);
	
	Hydroblock20Parser *hbparser = new Hydroblock20Parser(printer);

	hbparser->parse(gnssFilename, imuFilename, sonarFilename);
	
	std::cout << std::setprecision(12);
	std::cout << std::fixed;
	
	//Lever arm
    Eigen::Vector3d leverArm;
    leverArm << 0.0,0.0,0.0;

    //Boresight
    Attitude boresightAngles(0.0,0.0,0.0,0.0);
    Eigen::Matrix3d boresight;
    Boresight::buildMatrix(boresight,boresightAngles);
    
    //Do the georeference dance
    printer.georeference(leverArm, boresight, svps.getSvps());

    delete hbparser;

	return 0;
	
	
}
