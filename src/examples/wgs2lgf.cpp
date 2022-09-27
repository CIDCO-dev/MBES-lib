#include <iostream>
#include <vector>
#include <numeric>
#include <string.h>
#include <sstream>
#include "../math/CoordinateTransform.hpp"
#include "../Position.hpp"

int main(int argc, const char* argv[]) {
     
    if (argc != 2) {
    
    	if(argc >=2 && (strcasecmp(argv[1], "enu") != 0 && strcasecmp(argv[1], "ned") != 0)){
    		std::cerr<<"Frame must be NED or ENU"<<std::endl;
    	}
		std::cerr << "cat FILE | ./wgs2enu [ned|enu]"<<std::endl;
		return -1;
	}
	
	
	double lon;
	double lat;
	double ellipsoidalHeight;
	double latsMean;
	double lonsMean;
	double ellipsoidalHeightsMean;
	std::string line;
	std::vector<double> lats, lons, ellipsoidalHeights;
	
	while ( std::getline( std::cin, line ) ) {
		
	    std::istringstream stream( line );

	    if ( stream >> lon >> lat >> ellipsoidalHeight) { // 3 == sscanf(line.c_str(), "%f %f %f", &valueX, &valueY, &valueZ
			try{
				lats.push_back(lat);
				lons.push_back(lon);
				ellipsoidalHeights.push_back(ellipsoidalHeight);
				latsMean += lat;
				lonsMean += lon;
				ellipsoidalHeightsMean += ellipsoidalHeight;		
				
			}
			catch(std::exception & e){
				std::cerr <<  e.what() << std::endl;
			}
		}
	}
	
	if(lats.size() != lons.size() || lons.size() != ellipsoidalHeights.size() ){
	std::cerr<<"parsing error \n";
	return -1;
	}
	std::cerr << "[+] " << lats.size() << " lines read \n"; 


	latsMean /= (double)lats.size();
	lonsMean /=  (double)lons.size();
	ellipsoidalHeightsMean /= (double)ellipsoidalHeights.size();
	
	
	Eigen::MatrixXd ecefPoints(3, lats.size());
	for(int i = 0; i<lats.size(); ++i){
	
		Eigen::Vector3d positionECEF(0,0,0);
		Position position(0,lats.at(i), lons.at(i), ellipsoidalHeights.at(i));
		CoordinateTransform::getPositionECEF(positionECEF, position);
		
		ecefPoints(0, i) = positionECEF(0);
		ecefPoints(1, i) = positionECEF(1);
		ecefPoints(2, i) = positionECEF(2);
	}
	
	
	Eigen::Vector3d positionECEF(0,0,0);
	Position position(0,latsMean, lonsMean, ellipsoidalHeightsMean);
	CoordinateTransform::getPositionECEF(positionECEF, position);
	
	Eigen::Vector3d centroid(positionECEF(0), positionECEF(1), positionECEF(2));

	
	Eigen::Matrix3d ecef2lgf;
	if(strcasecmp(argv[1], "enu") == 0 ){
	
		ecef2lgf << -sin(lonsMean*D2R), cos(lonsMean*D2R), 0,
					-sin(latsMean*D2R)*cos(lonsMean*D2R), -sin(latsMean*D2R)*sin(lonsMean*D2R), cos(latsMean*D2R),
					cos(latsMean*D2R)*cos(lonsMean*D2R), cos(latsMean*D2R)*sin(lonsMean*D2R), sin(latsMean*D2R);
	}
	else if(strcasecmp(argv[1], "ned") == 0 ){
		ecef2lgf << -sin(latsMean*D2R)*cos(lonsMean*D2R), -sin(latsMean*D2R) * sin(lonsMean*D2R), cos(latsMean*D2R),
					-sin(lonsMean*D2R), cos(lonsMean*D2R), 0,
					-cos(latsMean*D2R)*cos(lonsMean*D2R), -cos(latsMean*D2R)*sin(lonsMean*D2R), -sin(latsMean*D2R); 
	}
	else{
		std::cerr<<"Frame must be NED or ENU"<<std::endl;
		return -1;
	}
	
	Eigen::MatrixXd lgfPoints = ecef2lgf * (ecefPoints.colwise() - centroid);
	
	std::cout.precision(20);
	for(int i =0; i<lgfPoints.cols(); ++i){
		std::cout<<lgfPoints(0, i)<<" "<<lgfPoints(1, i)<<" "<<lgfPoints(2, i)<< std::endl;
	}
	
	
	return 0;
}
