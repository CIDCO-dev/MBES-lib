#ifndef EEG_CPP
#define EEG_CPP

#include <cstdio>
#include <cstdlib>
#include <string>
#include <iostream>
#include <thread>



#include <algorithm>			// for max
#include <initializer_list>		// To use inside function max


#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


// #include "thirdParty/MBES-lib/src/DatagramGeoreferencer.hpp"
// #include "thirdParty/MBES-lib/src/datagrams/DatagramParserFactory.hpp"
// #include "thirdParty/MBES-lib/src/utils/StringUtils.hpp"

#include "../../../DatagramGeoreferencer.hpp"
#include "../../../datagrams/DatagramParserFactory.hpp"
#include "../../../utils/StringUtils.hpp"
#include "../../../math/Boresight.hpp"


void printUsage(){
	//TODO: better synopsis
	printf("eeg file1\n");
	exit(1);
}

class PointCloudGeoreferencer : public DatagramGeoreferencer{

	public:
		PointCloudGeoreferencer( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ) 
			: cloud( cloud ) {

		}

		virtual ~PointCloudGeoreferencer() {}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(){
			cloud->width = (int) cloud->points.size();
			cloud->height = (int) 1;

			return cloud;
		};

		virtual void processGeoreferencedPing(Eigen::Vector3d & ping,uint32_t quality,int32_t intensity){

			if(quality >= 3 && ping(0) > -6.29301e+8 && ping(1) > -6.29301e+8){

				pcl::PointXYZRGB point;

				point.x = ping(0);
				point.y = ping(1);
				point.z = ping(2);

				// White points
				// uint32_t rgb = (static_cast<uint32_t>(255) << 16 | static_cast<uint32_t>(255) << 8 
				//   | static_cast<uint32_t>(255));

				// Blue points
				uint32_t rgb = (static_cast<uint32_t>(0) << 16 | static_cast<uint32_t>(0) << 8 
					| static_cast<uint32_t>(255));

				point.rgb = *reinterpret_cast<float*>(&rgb);

				cloud->push_back(point);
			}
		}


	private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
};

// void loadCloudFromFile(std::string & fileName,PointCloudGeoreferencer & cloud){
// 	DatagramParser * parser = DatagramParserFactory::build(fileName,cloud);
//
//	parser->parse(fileName);
//
//	delete parser;
//}

int main(int argc, char ** argv){
	//Check CLI parameters for filenames
	// if(argc != 3){
	if(argc != 2){
		printUsage();
	}

	
	std::string filename1(argv[1]);


	//TODO: pass as CLI parameter
	Eigen::Vector3d	leverArm;
	leverArm <<  0, 0, 0;

	//Get point clouds from files
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);

	PointCloudGeoreferencer line1(cloud1);


	DatagramParser * parser = nullptr;


	std::cout << "\nReading sonar file" << std::endl;

	std::ifstream inFile;
	inFile.open( filename1 );


	if ( ends_with( filename1.c_str(),".all" ) )
	{
		parser = new KongsbergParser( line1 );
	}
	else if ( ends_with( filename1.c_str(),".xtf") )
	{
		parser = new XtfParser( line1 );
	}
	else if ( ends_with( filename1.c_str(),".s7k") )
	{
		parser = new S7kParser( line1 );
	}


	parser->parse( filename1 );


	//loadCloudFromFile(filename1,line1);

	std::cout << "\nGeoreferencing point cloud" << std::endl;

	// line1.georeference(leverArm);

	Attitude boresightAngles( 0, 0, 0, 0 ); //Attitude boresightAngles(0,roll,pitch,heading);
	Eigen::Matrix3d boresight;
	Boresight::buildMatrix( boresight, boresightAngles );

	line1.georeference( leverArm , boresight  );


	//Display point cloud stats
	std::cout << "Line 1: " << line1.getCloud()->points.size() << " points loaded" << std::endl;

	pcl::PointXYZRGB minPt, maxPt;
	pcl::getMinMax3D (*line1.getCloud(), minPt, maxPt);

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
	viewer->setBackgroundColor ( 1.0, 1.0, 1.0 ); // doubles between 0.0 and 1.0			  
	
	
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(line1.getCloud());

	viewer->addPointCloud<pcl::PointXYZRGB> (line1.getCloud(), rgb, "sample cloud");

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");


	// viewer->addCoordinateSystem (1.0);


	// viewer->setPosition( 300, 100 ); // Position of the window on the screen
	viewer->setSize( 1800, 1200 );		// Size of the window on the screen
	

	viewer->initCameraParameters();

	viewer->resetCameraViewpoint("sample cloud");


	double extentx = maxPt.x - minPt.x;
	double extenty = maxPt.y - minPt.y;
	double extentz = maxPt.z - minPt.z;		

	double maxExtent = std::max( { extentx, extenty, extentz } );

	cout << "\nmaxExtent: " << maxExtent << endl;

	// Focus position of the camera
	// Center of the point cloud 
	double view_x = ( minPt.x + maxPt.x ) / 2;;
	double view_y = ( minPt.y + maxPt.y ) / 2;;
	double view_z = ( minPt.z + maxPt.z ) / 2;;

	// Position of the camera
	double pos_x = view_x;
	double pos_y = view_y - maxExtent;
	double pos_z = view_z;

	// A vector in space indicating the direction of the top of the camera window plane
	double up_x = 0;
	double up_y = 0;
	double up_z = 1;

	
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


	viewer->setCameraPosition( pos_x, pos_y, pos_z, view_x, view_y, view_z, up_x, up_y, up_z );


	pcl::visualization::Camera cam; 

	viewer->getCameraParameters(cam);

	double cameraToFocalDistance = 0;

	for ( int countAxis = 0; countAxis < 3; countAxis++ ) 
		cameraToFocalDistance += ( cam.focal[ countAxis ] - cam.pos[ countAxis ] )
								* ( cam.focal[ countAxis ] - cam.pos[ countAxis ] );
	
	cameraToFocalDistance = sqrt( cameraToFocalDistance );

	viewer->setCameraClipDistances( 0.001, cameraToFocalDistance + 2 * maxExtent );



	// viewer->setCameraFieldOfView( 1.7150 );


	viewer->addCoordinateSystem( maxExtent/5.0, view_x, view_y, view_z );


	// http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
	cout << "\n\nTo exit the viewer application, press q.\n"
		<< "Press r to centre and zoom the viewer so that the entire cloud is visible.\n"
		<< "Use the mouse to rotate the viewpoint by clicking and dragging.\n"
		<< "You can use the scroll wheel, or right-click and drag up and down, to zoom in and out.\n"
		<< "Middle-clicking and dragging will move the camera.\n\n";


	int count = 0;

	while (!viewer->wasStopped()){


		viewer->getCameraParameters(cam);

		double cameraToFocalDistance = 0;

		for ( int countAxis = 0; countAxis < 3; countAxis++ ) 
			cameraToFocalDistance += ( cam.focal[ countAxis ] - cam.pos[ countAxis ] )
									* ( cam.focal[ countAxis ] - cam.pos[ countAxis ] );
				
		cameraToFocalDistance = sqrt( cameraToFocalDistance );


		viewer->setCameraClipDistances( 0.001, cameraToFocalDistance + 2* maxExtent );


		if ( count % 50 == 0 )
		{			
			cout << "Cam: " << endl 
						<< " - pos: (" << cam.pos[0] << ", " << cam.pos[1] << ", " << cam.pos[2] << ")" << endl 
						<< " - view: (" << cam.view[0] << ", " << cam.view[1] << ", " << cam.view[2] << ")" << endl
						<< " - focal: (" << cam.focal[0] << ", " << cam.focal[1] << ", "<< cam.focal[2] << ")" << endl
						<< " - clip: (" << cam.clip[0] << ", " << cam.clip[1] << ")" << endl
						<< " - fovy: (" << cam.fovy << ")" << endl
						<< " - window_size: (" << cam.window_size[0] << ", " << cam.window_size[1] << ")" << endl
						<< " - window_pos: (" << cam.window_pos[0] << ", " << cam.window_pos[1] << ")" << endl
						<< "- Camera to Focal: " << cameraToFocalDistance << endl;

			count = 0;
		}


		viewer->spinOnce (100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		count++;
	}
}

#endif
