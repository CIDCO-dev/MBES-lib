
#include <fstream>

/*
*  Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/*
* \author ?
*/

#ifndef GEOREFERENCE_CPP
#define GEOREFERENCE_CPP

#include <Eigen/Dense>
#include "../DatagramGeoreferencer.hpp"
#include "../datagrams/DatagramParserFactory.hpp"
#include <iostream>
#include <string>
#include "../utils/Exception.hpp"
#include "../math/Boresight.hpp"

using namespace std;

/**Writes the usage information about the program*/
void printUsage(){
	std::cerr << "\n\
	NAME\n\n\
	georeference - Produit un nuage de points d'un fichier de datagrammes multifaisceaux\n\n\
	SYNOPSIS\n \
	georeference [-x lever_arm_x] [-y lever_arm_y] [-z lever_arm_z] [-r roll_angle] [-p pitch_angle] [-h heading_angle] fichier\n\n\
	DESCRIPTION\n \
	-L Use a local geographic frame (NED)\n \
	-T Use a terrestrial geographic frame (WGS84 ECEF)\n\n \
	Copyright 2017-2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés" << std::endl;
	exit(1);
}


/**
* Declares the parser depending on argument received
*
* @param argc number of argument
* @param argv value of the arguments
*/
int main (int argc , char ** argv){

	#ifdef __GNU__
	setenv("TZ", "UTC", 1);
	#endif
	#ifdef _WIN32
	putenv("TZ");
	#endif
	if(argc < 2)
	{
		printUsage();
	}
	else
	{
		std::string fileName(argv[argc-1]);

		//Lever arm
		double leverArmX = 0.0;
		double leverArmY = 0.0;
		double leverArmZ = 0.0;

		//Boresight
		double roll     = 0.0;
		double pitch    = 0.0;
		double heading  = 0.0;

		//Georeference method
		Georeferencing * georef;

		int index;

		while((index=getopt(argc,argv,"x:y:z:r:p:h:LT"))!=-1)
		{
			switch(index)
			{
				case 'x':
				if(sscanf(optarg,"%lf", &leverArmX) != 1)
				{
					std::cerr << "Invalid lever arm X offset (-x)" << std::endl;
					printUsage();
				}
				break;

				case 'y':
				if (sscanf(optarg,"%lf", &leverArmY) != 1)
				{
					std::cerr << "Invalid lever arm Y offset (-y)" << std::endl;
					printUsage();
				}
				break;

				case 'z':
				if (sscanf(optarg,"%lf", &leverArmZ) != 1)
				{
					std::cerr << "Invalid lever arm Z offset (-z)" << std::endl;
					printUsage();
				}
				break;

				case 'r':
				if (sscanf(optarg,"%lf", &roll) != 1)
				{
					std::cerr << "Invalid roll angle offset (-p)" << std::endl;
					printUsage();
				}
				break;

				case 'h':
				if (sscanf(optarg,"%lf", &heading) != 1)
				{
					std::cerr << "Invalid heading angle offset (-P)" << std::endl;
					printUsage();
				}
				break;

				case 'p':
				if (sscanf(optarg,"%lf", &pitch) != 1)
				{
					std::cerr << "Invalid pitch angle offset (-t)" << std::endl;
					printUsage();
				}
				break;

				case 'L':
				georef = new GeoreferencingLGF();
				break;

				case 'T':
				georef = new GeoreferencingTRF();
				break;
			}
		}

		if(georef == NULL){
			std::cerr << "No georeferencing method defined (-L or -T)" << std::endl;
			printUsage();
		}

		try
		{
			DatagramParser * parser = NULL;
			DatagramGeoreferencer  printer(*georef);

			std::cerr << "Decoding " << fileName << std::endl;
			std::ifstream inFile;
			inFile.open(fileName);
			if (inFile){
				parser = DatagramParserFactory::build(fileName,printer);
			}
			else
			{
				throw new Exception("File not found");
			}
			parser->parse(fileName);
			std::cout << std::setprecision(6);
			std::cout << std::fixed;

			//Lever arm
			Eigen::Vector3d leverArm;
			leverArm << leverArmX,leverArmY,leverArmZ;

			//Boresight
			Attitude boresightAngles(0,roll,pitch,heading);
			Eigen::Matrix3d boresight;
			Boresight::buildMatrix(boresight,boresightAngles);

			//Do the georeference dance
			//TODO: get SVP file from CLI
			printer.georeference(leverArm,boresight,NULL);

			delete parser;
		}
		catch(Exception * error)
		{
			std::cerr << "Error while parsing " << fileName << ": " << error->getMessage() << std::endl;
		}
	}
}

#endif
