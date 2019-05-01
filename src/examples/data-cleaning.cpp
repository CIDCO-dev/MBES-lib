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
using namespace std;

/**Show the usage information about data-cleaning*/
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

/*!
 * \brief Point filter class
 */
class PointFilter{
   public:
       
       /**Create a point filter*/
	PointFilter(){

	}

        /**Destroy the point filter*/
	~PointFilter(){

	}
        
        /**
         * return true if we remove this point
         * 
         * @param microEpoch timestamp of the point
         * @param x x position of the point
         * @param y y position of the point
         * @param z z position of the point
         * @param quality quality of the point
         * @param intensity intensity of the point
         */
	virtual bool filterPoint(uint64_t microEpoch,double x,double y,double z, uint32_t quality,uint32_t intensity) = 0;
};

/*!
 * \brief Quality filter class extend of the Point filter class
 */
class QualityFilter : public PointFilter{
   public:

       /**
        * Create a quality filter
        * 
        * @param minimumQuality the minimal quality accepted
        */
	QualityFilter(int minimumQuality) : minimumQuality(minimumQuality){

	}

        /**Destroy the quality filter*/
	~QualityFilter(){

	}

        /**
         * return true if the quality receive is low then the minimum accepted
         * 
         * @param microEpoch timestamp of the point
         * @param x x position of the point
         * @param y y position of the point
         * @param z z position of the point
         * @param quality quality of the point
         * @param intensity intensity of the point
         */
	bool filterPoint(uint64_t microEpoch,double x,double y,double z, uint32_t quality,uint32_t intensity){
		return quality < minimumQuality;
	}

  private:
        
      /**minimal quality accepted*/
	unsigned int minimumQuality;

};

/*!
 * \brief Intensity filter class extend of the Point filter class
 */
class IntensityFilter : public PointFilter{
   public:

       /**
        * Create a intensity filter
        * 
        * @param minimumIntensity the minimal intensity accepted
        */
	IntensityFilter(int minimumIntensity) : minimumIntensity(minimumIntensity){

	}

        /**Destroy the intensity filter*/
	~IntensityFilter(){

	}

        /**
         * return true if the intensity receive is low then the minimum accepted
         * 
         * @param microEpoch timestamp of the point
         * @param x x position of the point
         * @param y y position of the point
         * @param z z position of the point
         * @param quality quality of the point
         * @param intensity intensity of the point
         */
	bool filterPoint(uint64_t microEpoch,double x,double y,double z, uint32_t quality,uint32_t intensity){
		return intensity < minimumIntensity;
	}

  private:
        
      /**minimal intensity accepted*/
	unsigned int minimumIntensity;

};

/*!
 * \brief Insane position filter class extend of the Point filter class
 */
class InsanePositionFilter : public PointFilter{
   public:

       /**
        * Create a insane position filter
        */
	InsanePositionFilter(){

	}

        /**Destroy the insane position filter*/
	~InsanePositionFilter(){

	}

        /**
         * return true if the position receive seem insane
         * 
         * @param microEpoch timestamp of the point
         * @param x x position of the point
         * @param y y position of the point
         * @param z z position of the point
         * @param quality quality of the point
         * @param intensity intensity of the point
         */
	bool filterPoint(uint64_t microEpoch,double x,double y,double z, uint32_t quality,uint32_t intensity){
            bool insaneX = ((x>1.00*100000000)||(x<-1.00*100000000));
            bool insaneY = ((y>1.00*100000000)||(y<-1.00*100000000));
            bool insaneZ = ((z>1.00*100000000)||(z<-1.00*100000000));
            return (insaneX||insaneY||insaneZ);
	}
};

/**
 * Filter all point receive in the terminal
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
                        std::cerr << "Error: parameter QualityFilter invalid" << std::endl;
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
                        std::cerr << "Error: parameter IntensityFilter invalid" << std::endl;
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
            uint64_t microEpoch;
            double x,y,z;
            uint32_t quality;
            uint32_t intensity;

            if(sscanf(line.c_str(),"%lu %lf %lf %lf %d %d",&microEpoch,&x,&y,&z,&quality,&intensity)==6){
		bool doFilter = false;

			//Apply filter chain
		for(auto i = filters.begin();i!= filters.end();i++){
                    if( (*i)->filterPoint(microEpoch,x,y,z,quality,intensity) ){
			doFilter = true;
                        break;
                    }
		}

		if(!doFilter){
                    printf("%lu %.6lf %.6lf %.6lf %d %d\r\n",microEpoch,x,y,z,quality,intensity);
		}
            }
            else{
		std::cerr << "Error at line " << lineCount << std::endl;
            }
                
            lineCount++;
        }
    }
#endif