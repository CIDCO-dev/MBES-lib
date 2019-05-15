/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/*
* \author ?
*/

#ifndef DATACLEANING_CPP
#define DATACLEANING_CPP

#include <cstdio>
#include <string>
#include <iostream>
#include <list>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include "../math/Interpolation.hpp"
#include "../filter/QualityFilter.hpp"
#include "../filter/IntensityFilter.hpp"
#include "../filter/InsanePositionFilter.hpp"

using namespace std;

/**Shows the usage information about data-cleaning*/
void printUsage(){
	std::cerr << "\n\
  NAME\n\n\
     data-cleaning - Filtre les points d'un nuage\n\n\
  SYNOPSIS\n \
	   data-cleaning [-q QualityFilter] [-i IntensityFilter]\n\n\
  DESCRIPTION\n\n \
  Copyright 2017-2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés" << std::endl;
	exit(1);
}

/**
 * Filter all points received on standard input
 *
 * @param argc number of parameter
 * @param argv value of the parameters
 */
int main(int argc,char** argv){

	std::string line;

	//Filter chain
	std::list<PointFilter *> filters;
        filters.push_back(new InsanePositionFilter());

	//TODO: load desired filters and parameters from command line
        int index;
        int quality;
        int intensity;
        while((index=getopt(argc,argv,"q:i:"))!=-1)
        {
            switch(index)
            {
                case 'q':
                    if(sscanf(optarg,"%d", &quality) != 1)
                    {
                        std::cerr << "Error: -q invalid quality parameter" << std::endl;
                        printUsage();
                    }
                    else
                    {
                        filters.push_back(new QualityFilter(quality));
                    }
                break;

                case 'i':
                    if(sscanf(optarg,"%d", &intensity) != 1)
                    {
                        std::cerr << "Error: -i invalid intensity parameter" << std::endl;
                        printUsage();
                    }
                    else
                    {
                        filters.push_back(new IntensityFilter(intensity));
                    }
                break;
            }
        }
        unsigned int lineCount = 1;
        while((std::getline(std::cin,line))&&(line!="0")){
            double x,y,z;
            uint32_t quality;
            uint32_t intensity;

            if(sscanf(line.c_str(),"%lf %lf %lf %d %d",&x,&y,&z,&quality,&intensity)==5){
		bool doFilter = false;

			//Apply filter chain
		for(auto i = filters.begin();i!= filters.end();i++){
                    if( (*i)->filterPoint(0,x,y,z,quality,intensity) ){
			doFilter = true;
                        break;
                    }
		}

		if(!doFilter){
                    printf("%.6lf %.6lf %.6lf %d %d\n",x,y,z,quality,intensity);
		}
            }
            else{
		std::cerr << "Error at line " << lineCount << std::endl;
            }
            lineCount++;
        }
    }
#endif
