/*
*  Copyright 2022 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

#include <iostream>
#include <vector>
#include <numeric>
#include <string.h>
#include <sstream>
#include "../math/CoordinateTransform.hpp"
#include "../Position.hpp"


int main(int argc, const char* argv[]) {
     
    if (argc != 5) {
    
    	if(argc >=5 && (strcasecmp(argv[1], "enu") != 0 && strcasecmp(argv[1], "ned") != 0)){
    		std::cerr<<"Frame must be NED or ENU"<<std::endl;
    	}
		std::cerr << "cat FILE | ./wgs2lgf [ned|enu]"<<std::endl;
		return -1;
	}
	
	double centroidLat = std::stod(argv[2]);
	double centroidLon = std::stod(argv[3]);
	double CentroidEllipsoidalHeight = std::stod(argv[4]);
		
	double lon;
	double lat;
	double ellipsoidalHeight;
	std::string line;
	std::vector<double> lats, lons, ellipsoidalHeights;
	
	while ( std::getline( std::cin, line ) ) {
		
	    std::istringstream stream( line );

	    if ( stream >> lat >> lon >> ellipsoidalHeight) { 
			try{
				lats.push_back(lat);
				lons.push_back(lon);
				ellipsoidalHeights.push_back(ellipsoidalHeight);		
				
			}
			catch(std::exception & e){
				std::cerr <<  e.what() << std::endl;
			}
		}
	}
	
	if(lats.size() != lons.size() ){
	std::cerr<<"parsing error \n";
	return -1;
	}
	std::cerr << "[+] " << lats.size() << " lines read \n";
	
	// Create Matrix of ecef points
	Eigen::MatrixXd ecefPoints(3, lats.size());
	for(int i = 0; i<lats.size(); ++i){
	
		Eigen::Vector3d positionECEF(0,0,0);
		Position position(0,lats.at(i), lons.at(i), 0.0);
		CoordinateTransform::getPositionECEF(positionECEF, position);
		
		ecefPoints(0, i) = positionECEF(0);
		ecefPoints(1, i) = positionECEF(1);
		ecefPoints(2, i) = positionECEF(2);
	}
	
	lats.clear();
	lons.clear();
	//ellipsoidalHeights.clear();
	
	// Create ecef centroid vector
	Eigen::Vector3d positionECEF(0,0,0);
	Position position(0, centroidLat, centroidLon, CentroidEllipsoidalHeight);
	CoordinateTransform::getPositionECEF(positionECEF, position);
	Eigen::Vector3d centroid(positionECEF(0), positionECEF(1), positionECEF(2));

	// Create ENU rotation matrix 
	Eigen::Matrix3d ecef2lgf;
	if(strcasecmp(argv[1], "enu") == 0 ){
	
		ecef2lgf << -sin(centroidLon*D2R), cos(centroidLon*D2R), 0,
					-sin(centroidLat*D2R)*cos(centroidLon*D2R), -sin(centroidLat*D2R)*sin(centroidLon*D2R), cos(centroidLat*D2R),
					cos(centroidLat*D2R)*cos(centroidLon*D2R), cos(centroidLat*D2R)*sin(centroidLon*D2R), sin(centroidLat*D2R);
	}
	// Create NED rotation matrix
	else if(strcasecmp(argv[1], "ned") == 0 ){
		ecef2lgf << -sin(centroidLat*D2R)*cos(centroidLon*D2R), -sin(centroidLat*D2R) * sin(centroidLon*D2R), cos(centroidLat*D2R),
					-sin(centroidLon*D2R), cos(centroidLon*D2R), 0,
					-cos(centroidLat*D2R)*cos(centroidLon*D2R), -cos(centroidLat*D2R)*sin(centroidLon*D2R), -sin(centroidLat*D2R); 
	}
	else{
		std::cerr<<"Frame must be NED or ENU"<<std::endl;
		return -1;
	}
	// Apply magic
	Eigen::MatrixXd lgfPoints = ecef2lgf * (ecefPoints.colwise() - centroid);
	
	std::cout.precision(20);
	for(int i =0; i<lgfPoints.cols(); ++i){
		std::cout<<lgfPoints(0, i)<<" "<<lgfPoints(1, i)<<" "<<ellipsoidalHeights.at(i)<< std::endl;
	}
	
	
	
	return 0;
}
