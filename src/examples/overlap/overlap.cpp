/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

 /*
 * \author Christian Bouchard
 */


#include <iostream>
#include <cstdint>

#include <sstream>

#include <thread>

#include <vector>

#include <utility>      // std::pair, std::make_pair


#include <chrono>


#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>


#include "../../PointCloudGeoreferencer.hpp"

#include "../../svp/SoundVelocityProfile.hpp"

#include "../../math/Boresight.hpp"

#include "../../utils/StringUtils.hpp"
#include "../../utils/Exception.hpp"

#include "../../HullOverlap.hpp"

#include "../smallUtilityFunctions.hpp"



int main( int argc, char* argv[] )
{
    std::cout << "\nStart of function main()\n" << std::endl;    


    // Time Stamp
    std::chrono::high_resolution_clock::time_point tStart = std::chrono::high_resolution_clock::now();  


    if ( argc != 7 && argc != 9 )
    {           
        std::cout << "\n\nThere should be 6 or 8 arguments after the program name\n\n" << endl;
        exit( 100 );
    }

    // Arguments:
    // 1: fileNameLine1
    // 2: fileNameLine2 

    // 3: Projection plane a, ax + by + cz + d = 0;
    // 4: Projection plane b
    // 5: Projection plane c
    // 6: Projection plane d

    // 7: Concave hull algorithm alpha for line 1
    // 8: Concave hull algorithm alpha for line 2


    // // Display all arguments
    // std::cout << "argc = " << argc << std::endl;
    // for(int i = 0; i < argc; i++)
    //     std::cout << "argv[" << i << "] = \"" << argv[i] << "\"" << std::endl;
    
    // std::cout << endl;


    // TODO: leverArm as CLI parameter

    // TODO: boresight as CLI parameter

    // TODO: get SVP from CLI

    std::string fileNameLine1( argv[ 1 ] );
    std::string fileNameLine2( argv[ 2 ] );


    // Plane in which to find the hulls and overlap
    // ax + by + cz + d = 0;

    // Initialize to dummy values
    double a = 0;
    double b = 0;
    double c = 1;
    double d = 0;

    // Get actual values from CLI
    convertStringToDouble( argv[ 3 ], a, "Plane parameter 'a'" );
    convertStringToDouble( argv[ 4 ], b, "Plane parameter 'b'" );
    convertStringToDouble( argv[ 5 ], c, "Plane parameter 'c'" );
    convertStringToDouble( argv[ 6 ], d, "Plane parameter 'd'" );

    std::cout << "\na: " << a << "\n" 
        << "b: " << b << "\n"
        << "c: " << c << "\n"
        << "d: " << d << "\n" << std::endl;
    

    // Initialize to dummy values
    double alphaLine1 = 1.0;
    double alphaLine2 = 1.0;

    if ( argc == 9 )
    {
        // Get actual values from CLI
        convertStringToDouble( argv[ 7 ], alphaLine1, "alphaLine1", true );
        convertStringToDouble( argv[ 8 ], alphaLine2, "alphaLine2", true );
    }    


    std::cout << "alphaLine1: " << alphaLine1 << "\n" 
        << "alphaLine2: " << alphaLine2<< "\n" << std::endl;

 

    pcl::PointCloud<pcl::PointXYZ>::Ptr line1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr line2 (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector< pcl::PointCloud<pcl::PointXYZ >::Ptr > twoLines;
    twoLines.reserve( 2 );
    twoLines.push_back( line1 ) ;
    twoLines.push_back( line2 ) ;


    std::vector< std::string > twoFileNames;
    twoFileNames.reserve( 2 );
    twoFileNames.push_back( fileNameLine1 ) ;
    twoFileNames.push_back( fileNameLine2 ) ;

    for ( int count = 0; count < 2; count++ )
    {
        // If file name ends in .txt: point cloud X, Y, Z
        if ( ends_with( twoFileNames[ count ].c_str(),".txt" ) )
        {
            readTextFileIntoPointCloud( twoFileNames[ count ], twoLines[ count ] );
        }
        else // Georeference
        {
            try
            {
                //TODO: pass as CLI parameter
                Eigen::Vector3d	leverArm;
                leverArm <<  0, 0, 0;

                //TODO: pass as CLI parameter
                Attitude boresightAngles( 0, 0, 0, 0 ); //Attitude boresightAngles(0,roll,pitch,heading);
                Eigen::Matrix3d boresight;
                Boresight::buildMatrix( boresight, boresightAngles );                
                
                //TODO: get SVP from CLI


                readSonarFileIntoPointCloud( twoFileNames[ count ], twoLines[ count ], leverArm , boresight, NULL, false );

            }
            catch ( Exception * error )
            {
                cout << error->getMessage();

                exit( 1 );
            }


        }


        //Display point cloud stats
        std::cout << "\nLine #" << count + 1 << ": " << twoLines[ count ]->points.size() << " points loaded" << std::endl;

        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D( *twoLines[ count ] , minPt, maxPt);

        std::cout << setprecision( 15 );

        std::cout << "\nMax x: " << maxPt.x << std::endl;
        std::cout << "Min x: " << minPt.x << std::endl;

        std::cout << "\nMax y: " << maxPt.y << std::endl;
        std::cout << "Min y: " << minPt.y << std::endl;

        std::cout << "\nMax z: " << maxPt.z << std::endl;
        std::cout << "Min z: " << minPt.z << std::endl;

    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr line1InBothHulls (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr line2InBothHulls (new pcl::PointCloud<pcl::PointXYZ>);


    std::cout << "\n\nProcessing to find the overlap\n" << std::endl;

    HullOverlap hullOverlap( line1, line2, a, b, c, d, alphaLine1, alphaLine2 );


    std::pair< uint64_t, uint64_t > inBothHulls = hullOverlap.computePointsInBothHulls( line1InBothHulls, 
                                                                                                    line2InBothHulls );   

    std::cout << "Nb points line1 in both hulls: " << inBothHulls.first
        << "\nNb points line2 in both hulls: " << inBothHulls.second << "\n" << std::endl;


    std::chrono::high_resolution_clock::time_point tEnd = std::chrono::high_resolution_clock::now();
    cout << "\n\nTotal time: " << std::chrono::duration_cast<std::chrono::seconds>(tEnd - tStart).count() << "s" << endl;       

    // ------------------------------------ Visualization -----------------------------------------

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters();


    // Viewport for line 1 and 2 (in the x, y, z coordinate system they were in the files)
    int viewport0 = 0;
    viewer->createViewPort( 0.0, 0.0, 0.5, 1.0, viewport0 );    
	viewer->setBackgroundColor ( 1.0, 1.0, 1.0, viewport0 );		

    // Display line 1
    viewer->addPointCloud<pcl::PointXYZ> ( line1, "line1", viewport0 );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "line1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "line1");    

    // Display line 2
    viewer->addPointCloud<pcl::PointXYZ> ( line2, "line2", viewport0 );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "line2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "line2");



    // Viewport for line 1 and 2 part of both hulls
    int viewport1 = 0;
    viewer->createViewPort( 0.5, 0.0, 1.0, 1.0, viewport1 );    
	viewer->setBackgroundColor ( 1.0, 1.0, 1.0, viewport1 );		

    // Display points of line 1 in both hulls
    viewer->addPointCloud<pcl::PointXYZ> ( line1InBothHulls, "line1InBothHulls", viewport1 );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "line1InBothHulls");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "line1InBothHulls");    

    // Display points of line 2 in both hulls
    viewer->addPointCloud<pcl::PointXYZ> ( line2InBothHulls, "line2InBothHulls", viewport1 );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "line2InBothHulls");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "line2InBothHulls");

   
	// viewer->setPosition( 300, 100 ); // Position of the window on the screen
	// viewer->setSize( 1800, 1200 );		// Size of the window on the screen
	
	viewer->resetCameraViewpoint("Everything");

	// viewer->addCoordinateSystem( 10, 0, 0, 0 );

	// http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
	cout << "\n\nTo exit the viewer application, press q.\n"
		<< "Press r to centre and zin the projection planeoom the viewer so that the entire cloud is visible.\n"
		<< "Use the mouse to rotatein the projection plane the viewpoint by clicking and dragging.\n"
		<< "You can use the scroll in the projection planewheel, or right-click and drag up and down, to zoom in and out.\n"
		<< "Middle-clicking and dragging will move the camera.\n\n";

	while ( !viewer->wasStopped() ){

		viewer->spinOnce (100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}



    return 0;
}


