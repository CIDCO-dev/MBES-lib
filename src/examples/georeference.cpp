
#include <fstream>

/*
 *  Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */
#ifndef GEOREFERENCE_CPP
#define GEOREFERENCE_CPP

#include <Eigen/Dense>
#include "../DatagramGeoreferencer.hpp"
#include "../datagrams/DatagramParserFactory.hpp"
#include <iostream>
#include <string>
#include "../utils/Exception.hpp"


using namespace std;

/**Write the information about the georeference*/
void printUsage(){
	std::cerr << "\n\
  NAME\n\n\
     georeference - Produit un nuage de points d'un fichier de datagrammes multifaisceaux\n\n\
  SYNOPSIS\n \
	   georeference [-x lever_arm_x] [-y lever_arm_y] [-z lever_arm_z] fichier\n\n\
  DESCRIPTION\n\n \
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
	DatagramParser * parser = NULL;
	DatagramGeoreferencer  printer;

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
    double leverArmX = 0.0;
    double leverArmY = 0.0;
    double leverArmZ = 0.0;
    int index;
    while((index=getopt(argc,argv,"x:y:z:"))!=-1)
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
        }
    }
    try
    {
	std::cerr << "Decoding " << fileName << std::endl;
        std::ifstream inFile;
        inFile.open(fileName);
        if (inFile){
		parser = DatagramParserFactory::build(fileName,printer);
        }
        else
        {
            throw "File not found";
        }
        parser->parse(fileName);
	std::cout << std::setprecision(6);
	std::cout << std::fixed;

	Eigen::Vector3d leverArm;
	leverArm << leverArmX,leverArmY,leverArmZ;

        printer.georeference(leverArm);
    }
    catch(Exception * error)
    {
	std::cerr << "Error while parsing " << fileName << ": " << error << std::endl;
    }
    if(parser) delete parser;
}       
}

#endif
