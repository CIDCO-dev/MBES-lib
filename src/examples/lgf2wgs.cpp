/*
*  Copyright 2022 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

#include <iostream>
#include <vector>
#include <numeric>
#include <string.h>
#include <sstream>
#include "../math/CoordinateTransform.hpp"
#include "../math/CartesianToGeodeticFukushima.hpp"
#include "../Position.hpp"


int main(int argc, const char* argv[]) {
     
    if (argc != 5) {
    
    	if(argc >=5 && (strcasecmp(argv[1], "enu") != 0 && strcasecmp(argv[1], "ned") != 0)){
    		std::cerr<<"Frame must be NED or ENU"<<std::endl;
    	}
		std::cerr << "cat FILE | ./lgf2wgs [ned|enu] x y z"<<std::endl;
		return -1;
	}
	
	double centroidLat = std::stod(argv[2]);
	double centroidLon = std::stod(argv[3]);
	double CentroidEllipsoidalHeight = std::stod(argv[4]);
	
	double x;
	double y;
	double z;
	std::string line;
	std::vector<double> xs, ys, zs;
	
	while ( std::getline( std::cin, line ) ) {
		
	    std::istringstream stream( line );

	    if ( 3 == sscanf(line.c_str(), "%lf,%lf,%lf", &x, &y, &z)) { 
			try{
				xs.push_back(x);
				ys.push_back(y);
				zs.push_back(z);		
				
			}
			catch(std::exception & e){
				std::cerr <<  e.what() << std::endl;
			}
		}
	}
	
	if(xs.size() != ys.size() && ys.size() != zs.size()){
	std::cerr<<"parsing error \n";
	return -1;
	}
	std::cerr << "[+] " << xs.size() << " lines read \n";
	
	// Create Matrix of ecef points
	Eigen::MatrixXd lgfPoints(3, xs.size());
	for(int i = 0; i<xs.size(); ++i){
		
		lgfPoints(0, i) = xs.at(i);
		lgfPoints(1, i) = ys.at(i);
		lgfPoints(2, i) = zs.at(i);
	}
	
	xs.clear();
	ys.clear();
	zs.clear();
	
	// Create ecef centroid vector
	Eigen::Vector3d positionECEF(0,0,0);
	Position position(0, centroidLat, centroidLon, CentroidEllipsoidalHeight);
	CoordinateTransform::getPositionECEF(positionECEF, position);
	Eigen::Vector3d centroid(positionECEF(0), positionECEF(1), positionECEF(2));

	// Create ENU rotation matrix 
	Eigen::Matrix3d lgf2ecef;
	if(strcasecmp(argv[1], "enu") == 0 ){
		
		lgf2ecef << -sin(centroidLon*D2R), -sin(centroidLat*D2R)*cos(centroidLon*D2R), cos(centroidLat*D2R)*cos(centroidLon*D2R),
					cos(centroidLon*D2R), -sin(centroidLat*D2R)*sin(centroidLon*D2R), cos(centroidLat*D2R)*sin(centroidLon*D2R),
					0, cos(centroidLat*D2R), sin(centroidLat*D2R);
	}
	
	// Create NED rotation matrix
	else if(strcasecmp(argv[1], "ned") == 0 ){
	
		lgf2ecef << -sin(centroidLat*D2R)*cos(centroidLon*D2R), -sin(centroidLon*D2R), -cos(centroidLat*D2R)*cos(centroidLon*D2R),
					-sin(centroidLat*D2R) * sin(centroidLon*D2R), cos(centroidLon*D2R), -cos(centroidLat*D2R)*sin(centroidLon*D2R),
					cos(centroidLat*D2R), 0, -sin(centroidLat*D2R);
	}
	else{
		std::cerr<<"Frame must be NED or ENU"<<std::endl;
		return -1;
	}
	// Apply magic
	Eigen::MatrixXd ecefPoints = lgf2ecef * lgfPoints;
	ecefPoints = ecefPoints.colwise() + centroid;
	
	CartesianToGeodeticFukushima ecef2wgs(2);
	Eigen::Vector3d ecefPosition(0,0,0);
	Position positionGeographic(0,0,0,0);
	
	std::cout.precision(20);
	for(int i =0; i<ecefPoints.cols(); ++i){
		
		ecefPosition[0] = ecefPoints(0, i);
		ecefPosition[1] = ecefPoints(1, i);
		ecefPosition[2] = ecefPoints(2, i);
		
		ecef2wgs.ecefToLongitudeLatitudeElevation(ecefPosition , positionGeographic);
		
		std::cout<< positionGeographic.getVector()(0) << " " << positionGeographic.getVector()(1) << " " << 
					positionGeographic.getVector()(2) << std::endl;
		
	}
	
	
	
	return 0;
}
