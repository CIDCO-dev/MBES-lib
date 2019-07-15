/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

 /*
 * \author Guillaume Labbe-Morissette, Christian Bouchard
 */

#ifndef GEOREFPCLVIEWER_CPP
#define GEOREFPCLVIEWER_CPP

#ifdef _WIN32
#include "src/getopt.h"
#endif

#include <cstdio>
#include <cstdlib>
#include <string>
#include <iostream>
#include <thread>

#include <algorithm>			// For max
#include <initializer_list>		// To use as parameters to function max

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


#include "../../georeferencing/PointCloudGeoreferencer.hpp"

#include "../../utils/StringUtils.hpp"
#include "../../math/Boresight.hpp"

#include "smallUtilityFunctions.hpp"


/**Writes the usage information about the program*/
void printUsage(){
	// TODO: How to indicate that either -L or -T must be present?
	std::cerr << "\n\
	NAME\n\n\
	viewer - Displays the point cloud from a multibeam echosounder datagram file\n\n\
	SYNOPSIS\n \
	viewer [-x lever_arm_x] [-y lever_arm_y] [-z lever_arm_z] [-r roll_angle] [-p pitch_angle] [-h heading_angle] fichier\n\n\
	DESCRIPTION\n \
	-L Use a local geographic frame (NED)\n \
	-T Use a terrestrial geographic frame (WGS84 ECEF)\n\n \
	Copyright 2017-2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés\n" << std::endl;
	exit(1);
}


/**
* Declares the viewer depending on argument received
*
* @param argc number of argument
* @param argv value of the arguments
*/
int main(int argc, char ** argv){


	// TODO: the following is present in the example program "georeference.cpp", is it required here as well?
	// #ifdef __GNU__
	// setenv("TZ", "UTC", 1);
	// #endif
	// #ifdef _WIN32
	// putenv("TZ");
	// #endif


	//Check CLI parameters for filenames
	if(argc < 2){
		printUsage();
	}

	std::string filename( argv[ argc - 1 ] );

	// TODO: get SVP from CLI

	//Lever arm
	double leverArmX = 0.0;
	double leverArmY = 0.0;
	double leverArmZ = 0.0;

	//Boresight
	double roll     = 0.0;
	double pitch    = 0.0;
	double heading  = 0.0;

	bool LorTPresent = false;
	bool DoLGF = true;

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
				LorTPresent = true;
				DoLGF = true;
				break;

			case 'T':
				LorTPresent = true;
				DoLGF = false;
				break;
		}
	}

	if( LorTPresent == false ){
		std::cerr << "\nNo georeferencing method defined (-L or -T)" << std::endl;
		printUsage();
	}


	Eigen::Vector3d	leverArm( leverArmX, leverArmY, leverArmZ );

	Attitude boresightAngles( 0, roll,pitch,heading );
	Eigen::Matrix3d boresight;
	Boresight::buildMatrix( boresight, boresightAngles );



	//Get point clouds from files
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	try
	{
		readSonarFileIntoPointCloud( filename, cloud, leverArm , boresight, NULL, DoLGF );
	}
	catch ( Exception * error )
	{
		cout << error->getMessage();

		exit( 1 );
	}


	//Display point cloud stats
	std::cout << "Line: " << cloud->points.size() << " points loaded" << std::endl;

	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D ( *cloud, minPt, maxPt);

	std::cout << setprecision( 15 );

	std::cout << "\nMax x: " << maxPt.x << std::endl;
	std::cout << "Min x: " << minPt.x << std::endl;

	std::cout << "\nMax y: " << maxPt.y << std::endl;
	std::cout << "Min y: " << minPt.y << std::endl;

	std::cout << "\nMax z: " << maxPt.z << std::endl;
	std::cout << "Min z: " << minPt.z << std::endl;


	//Load viewer
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));


	// Black background
	// viewer->setBackgroundColor (0, 0, 0);

	// White background
	viewer->setBackgroundColor ( 1.0, 1.0, 1.0 ); // Doubles between 0.0 and 1.0


	viewer->addPointCloud<pcl::PointXYZ> ( cloud, "sample cloud" );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "sample cloud" ); // Blue points
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud" );


	// viewer->setPosition( 300, 100 ); // Position of the window on the screen
	viewer->setSize( 1800, 1200 );		// Size of the window on the screen

	viewer->initCameraParameters();

	viewer->resetCameraViewpoint("sample cloud");


	double extentx = maxPt.x - minPt.x;
	double extenty = maxPt.y - minPt.y;
	double extentz = maxPt.z - minPt.z;

	double maxExtent = std::max( { extentx, extenty, extentz } );

	cout << "\nmaxExtent: " << maxExtent << endl;


	// Center of the point cloud
	double view_x = ( minPt.x + maxPt.x ) / 2;
	double view_y = ( minPt.y + maxPt.y ) / 2;
	double view_z = ( minPt.z + maxPt.z ) / 2;

	// There are discrepancies between
	// pcl::visualization::PCLVisualizer   and     pcl::visualization::Camera
	// about what "view" is:

	// The position where the camera is focused:
	// With function void pcl::visualization::PCLVisualizer::setCameraPosition( ),
	// this is view_x, view_y, view_z
	// In class pcl::visualization::Camera, this is the array "focal[3]"

	// A vector in space indicating the direction of the top of the camera window plane:
	// With function void pcl::visualization::PCLVisualizer::setCameraPosition( ),
	// this is up_x, up_y, up_z
	// In class pcl::visualization::Camera, this is the array "view[3]"


	viewer->addCoordinateSystem( maxExtent / 5.0, view_x, view_y, view_z );


	// http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
	cout << "\n\nTo exit the viewer application, press q.\n"
	<< "Press r to centre and zoom the viewer so that the entire cloud is visible.\n"
	<< "Use the mouse to rotate the viewpoint by clicking and dragging.\n"
	<< "You can use the scroll wheel, or right-click and drag up and down, to zoom in and out.\n"
	<< "Middle-clicking and dragging will move the camera.\n\n";

	while ( !viewer->wasStopped() ){

		viewer->spinOnce (100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

}

#endif
