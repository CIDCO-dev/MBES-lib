/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/*
* \author Guillaume Labbe-Morissette
*/

#ifndef MAIN_CPP
#define MAIN_CPP

#include "../datagrams/DatagramParserFactory.hpp"
#include <iostream>
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"

/**Writes the usage information about the datagram-list*/
void printUsage(){
	std::cerr << "\n\
	NAME\n\n\
	sidescan-dump - Dumps sidescan data to an image file\n\n\
	SYNOPSIS\n \
	sidescan-dump file\n\n\
	DESCRIPTION\n\n \
	Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), All rights reserved" << std::endl;
	exit(1);
}

/*!
* \brief Datagram Printer class
*
* Extends DatagramEventHandler.
*/
class SidescanDataDumper : public DatagramEventHandler{
public:
        
	/**
	* Creates a SidescanDataDumper
	*/
	SidescanDataDumper(std::string & filename):filename(filename){

	}

	/**Destroys the SidescanDataDumper*/
	~SidescanDataDumper(){

	}

        void processSidescanData(unsigned int channel,std::vector<double> & data){
            
            if(channels.size() < channel+1){
                std::vector<std::vector<double>*> * u = new std::vector<std::vector<double>*>();
                channels.push_back(u);
            }
            
            std::vector<double> * v = new std::vector<double>();
            
            for(auto i=data.begin();i!=data.end();i++){
                //std::cerr << *i << " ";
                v->push_back(*i);
            }
            
            //std::cerr << std::endl;

            channels[channel]->push_back(v);           
        }
        
        void generateImages(){
            std::cerr << "Generating images for " << channels.size() << " channels" << std::endl;

            for(unsigned int i=0;i<channels.size();i++){
                std::stringstream ss;
                
                ss << filename <<  "-channel-" << i << ".jpg";
                std::cerr << "Channel " << i << std::endl;
                
                cv::Mat img(channels[i]->size(),channels[i]->at(0)->size(), CV_64F,cv::Scalar(70));
                
                std::cerr << "Rows: " << channels[i]->size() << " Cols: " << channels[i]->at(0)->size() << std::endl;                 
                
                for(unsigned int j=0;j<channels[i]->size();j++){ //j indexes rows
                    for(unsigned int k=0;k<channels[i]->at(j)->size();k++){ //k indexes cols 
                        img.at<double>(j, k, 0) = channels[i]->at(j)->at(k);
                    }
                }

		/*
		cv::namedWindow("Image", CV_WINDOW_AUTOSIZE);
		cv::imshow("Image",img);
		cv::waitKey(0);
		cv::destroyWindow("Image");
		*/
                cv::normalize(img,img,200000,0);
		cv::Mat I;
		img.convertTo(I, CV_8UC1);
		equalizeHist(I,I);
		imwrite(ss.str(), I);                
            }
        }
        
private:
    std::string filename;
    
    std::vector<  std::vector<std::vector<double> * > * > channels;
};

/**
* Declares the parser depending on argument received
*
* @param argc number of argument
* @param argv value of the arguments
*/
int main (int argc , char ** argv ){
	DatagramParser *    parser      = NULL;

        #ifdef __GNU__
	setenv("TZ", "UTC", 1);
	#endif
	#ifdef _WIN32
	putenv("TZ");
	#endif

	if(argc != 2){
		printUsage();
	}

	std::string fileName(argv[1]);

	try{
            	SidescanDataDumper  sidescan(fileName);

		std::cerr << "Decoding " << fileName << std::endl;

		parser = DatagramParserFactory::build(fileName,sidescan);

		parser->parse(fileName);
                
                sidescan.generateImages();
	}
	catch(std::exception * e){
		std::cerr << "Error while parsing " << fileName << ": " << e->what() << std::endl;
	}


	if(parser) delete parser;
}


#endif
