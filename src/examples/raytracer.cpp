/*
 * Copyright 2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
* \author Jordan McManus
*/

#ifndef MAIN_CPP
#define MAIN_CPP

#pragma comment(lib, "Ws2_32.lib")

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include "../georeferencing/DatagramRayTracer.hpp"
#include "../svp/CarisSvpFile.hpp"
#include "../datagrams/DatagramParserFactory.hpp"
#include "../svp/SvpSelectionStrategy.hpp"
#include "../svp/SvpNearestByTime.hpp"
#include "../svp/SvpNearestByLocation.hpp"
#include "../math/Boresight.hpp"

void printUsage(){
	std::cerr << "\n\
NAME\n\n\
	raytracer - Produces raytraced vectors for each beam \n\n\
SYNOPSIS\n \
	raytracer [-x lever_arm_x] [-y lever_arm_y] [-z lever_arm_z] [-r roll_angle] [-p pitch_angle] [-h heading_angle] [-d transducer_depth] [-s svp_file] [-S svpStrategy] file\n\n\
DESCRIPTION\n \
    This application will output the raytraced vector for each beam in the MBES file\n\n \
        -S choose one: nearestTime or nearestLocation\n\n \
Copyright 2017-2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés" << std::endl;
        
        exit(1);
}


/**
* @param argc number of argument
* @param argv value of the arguments
*/
int main (int argc , char ** argv ){
	

	#ifdef __GNU__
	setenv("TZ", "UTC", 1);
	#endif
	#ifdef _WIN32
	putenv("TZ");
	#endif

        

	if(argc < 2){
            printUsage();
	}
        
        std::string fileName(argv[argc-1]);
        
        //draft
        double transducerDraft = 0.0;

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
        SvpSelectionStrategy * svpStrategy = NULL;

	std::string	     svpFilename;
	CarisSvpFile svps;
        
        int index;
        while((index=getopt(argc,argv,"x:y:z:r:p:h:d:s:S:LTg"))!=-1)
        {
            switch(index)
            {
                case 'x':
                    if(sscanf(optarg,"%lf", &leverArmX) != 1)
                    {
                        std::cerr << "Invalid lever arm X offset (-x): " << leverArmX << std::endl;
                        printUsage();
                    }
               break;

                case 'y':
                    if (sscanf(optarg,"%lf", &leverArmY) != 1)
                    {
                        std::cerr << "Invalid lever arm Y offset (-y): " << leverArmY << std::endl;
                        printUsage();
                    }
                break;

                case 'z':
                    if (sscanf(optarg,"%lf", &leverArmZ) != 1)
                    {
                        std::cerr << "Invalid lever arm Z offset (-z): " << leverArmZ << std::endl;
                        printUsage();
                    }
                break;

                case 'r':
                    if (sscanf(optarg,"%lf", &roll) != 1)
                    {
                        std::cerr << "Invalid roll angle offset (-r): " << roll << std::endl;
                        printUsage();
                    }
                break;

                case 'h':
                    if (sscanf(optarg,"%lf", &heading) != 1)
                    {
                        std::cerr << "Invalid heading angle offset (-h): " << heading << std::endl;
                        printUsage();
                    }
                break;

                case 'p':
                    if (sscanf(optarg,"%lf", &pitch) != 1)
                    {
                        std::cerr << "Invalid pitch angle offset (-p): " << pitch << std::endl;
                        printUsage();
                    }
                break;
                
                case 'd':
                    if (sscanf(optarg, "%lf", &transducerDraft) != 1) {
                        std::cerr << "Invalid transducer draft (-d): " << transducerDraft << std::endl;
                        printUsage();
                    } else {
                        std::cerr << "Transducer draft is: " << transducerDraft << std::endl;
                    }
                break;

		case 's':
			svpFilename = optarg;
			if(!svps.readSvpFile(svpFilename)){
				std::cerr << "Invalid SVP file (-s): " << svpFilename << std::endl;
				printUsage();
			}
                        break;
                        
                case 'S':
			userSelectedStrategy = optarg;
                        if(userSelectedStrategy == "nearestLocation") {
                            std::cerr << "[+] Using nearest location sound velocity profile selection strategy" << std::endl;
                            svpStrategy = new SvpNearestByLocation();
                        } else if(userSelectedStrategy == "nearestTime") {
                            std::cerr << "[+] Using nearest location sound velocity profile selection strategy" << std::endl;
                            svpStrategy = new SvpNearestByTime();
                        } else {
                            std::cerr << "Invalid SVP strategy (-S): " << userSelectedStrategy << std::endl;
                            std::cerr << "Possible choices are:" << std::endl;
                            std::cerr << "-S nearestTime" << std::endl;
                            std::cerr << "-S nearestLocation" << std::endl;
                            printUsage();
                        }
                        break;
            }
        }
        
        DatagramRayTracer handler(*svpStrategy);
        
        DatagramParser * parser = NULL;

	try{
            std::cerr << "Decoding " << fileName << std::endl;

            parser = DatagramParserFactory::build(fileName,handler);

            parser->parse(fileName, true);

            std::cout << std::setprecision(12);
            std::cout << std::fixed;

            //Lever arm
            Eigen::Vector3d leverArm;
            leverArm << leverArmX,leverArmY,leverArmZ;

            //Boresight
            Attitude boresightAngles(0,roll,pitch,heading);
            Eigen::Matrix3d boresight;
            Boresight::buildMatrix(boresight,boresightAngles);
            
            //Do the georeference dance
            handler.raytrace(leverArm, boresight, svps.getSvps());
            
	}
	catch(const char * error){
		std::cerr << "Error whille parsing " << fileName << ": " << error << std::endl;
	}


	if(parser) delete parser;
}

#endif
