/*
 *  Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */
#ifndef GEOREFERENCE_CPP
#define GEOREFERENCE_CPP

#ifdef _WIN32
#include "../utils/getopt.h"
#pragma comment(lib, "Ws2_32.lib")
#endif

#include <fstream>
#include <Eigen/Dense>
#include "../georeferencing/DatagramGeoreferencer.hpp"
#include "../datagrams/DatagramParserFactory.hpp"
#include <iostream>
#include <string>
#include "../utils/Exception.hpp"
#include "../math/Boresight.hpp"
#include "../svp/CarisSVP.hpp"
#include "../svp/SvpSelectionStrategy.hpp"
#include "../svp/SvpNearestByTime.hpp"
#include "../svp/SvpNearestByLocation.hpp"

using namespace std;

/**Write the information about the program*/
void printUsage(){
	std::cerr << "\n\
NAME\n\n\
	georeference - Produces a georeferenced point cloud from binary multibeam echosounder datagrams files\n\n\
SYNOPSIS\n \
	georeference [-x lever_arm_x] [-y lever_arm_y] [-z lever_arm_z] [-r roll_angle] [-p pitch_angle] [-h heading_angle] [-s svp_file] [-S svpStrategy] file\n\n\
DESCRIPTION\n \
	-L Use a local geographic frame (NED)\n \
	-T Use a terrestrial geographic frame (WGS84 ECEF)\n \
        -S choose one: nearestTime or nearestLocation\n\n \
Copyright 2017-2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés" << std::endl;
	exit(1);
}

/**
  * declare the parser depending on argument receive
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
        
        //SVP strategy
        std::string userSelectedStrategy;
        SvpSelectionStrategy * svpStrategy;

        //Georeference method
        Georeferencing * georef;

	std::string	     svpFilename;
	CarisSVP svps;

        int index;

        while((index=getopt(argc,argv,"x:y:z:r:p:h:s:S:LT"))!=-1)
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
                        std::cerr << "Invalid roll angle offset (-r)" << std::endl;
                        printUsage();
                    }
                break;

                case 'h':
                    if (sscanf(optarg,"%lf", &heading) != 1)
                    {
                        std::cerr << "Invalid heading angle offset (-h)" << std::endl;
                        printUsage();
                    }
                break;

                case 'p':
                    if (sscanf(optarg,"%lf", &pitch) != 1)
                    {
                        std::cerr << "Invalid pitch angle offset (-p)" << std::endl;
                        printUsage();
                    }
                break;

		case 's':
			svpFilename = optarg;
			if(!svps.readSvpFile(svpFilename)){
				std::cerr << "Invalid SVP file (-s)" << std::endl;
				printUsage();
			}
                        break;
                        
                case 'S':
			userSelectedStrategy = optarg;
                        if(userSelectedStrategy == "nearestLocation") {
                            svpStrategy = new SvpNearestByLocation();
                        } else if(userSelectedStrategy == "nearestTime") {
                            svpStrategy = new SvpNearestByTime();
                        } else {
                            std::cerr << "Invalid SVP strategy (-S): " << userSelectedStrategy << std::endl;
                            std::cerr << "Possible choices are:" << std::endl;
                            std::cerr << "-S nearestTime" << std::endl;
                            std::cerr << "-S nearestLocation" << std::endl;
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
        
        if(svpStrategy == NULL){
            std::cerr << "[+] Using nearest in time sound velocity profile selection strategy" << std::endl;
            svpStrategy = new SvpNearestByTime();
        }

        try
        {
            DatagramParser * parser = NULL;
            DatagramGeoreferencer  printer(*georef, *svpStrategy);

            std::cerr << "[+] Decoding " << fileName << std::endl;
            std::ifstream inFile;
            inFile.open(fileName);
            if (inFile) {
                    parser = DatagramParserFactory::build(fileName,printer);
            }
            else
            {
                throw new Exception("File not found: << fileName");
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
            printer.georeference(leverArm, boresight, svps.getSvps());

            delete parser;
        }
        catch(Exception * error)
        {
            std::cerr << "[-] Error while parsing " << fileName << ": " << error->what() << std::endl;
        }
    }
}

#endif
