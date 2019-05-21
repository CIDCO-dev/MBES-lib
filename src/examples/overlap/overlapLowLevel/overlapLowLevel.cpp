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


#include "../../../DatagramGeoreferencer.hpp"
#include "../../../datagrams/DatagramParserFactory.hpp"

#include "../../../svp/SoundVelocityProfile.hpp"

#include "../../../math/Boresight.hpp"

#include "../../../utils/StringUtils.hpp"
#include "../../../utils/Exception.hpp"

#include "../../../HullOverlap.hpp"

#include "../smallUtilities.hpp"



void drawLinesFromPointToPoint( pcl::visualization::PCLVisualizer::Ptr viewer,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, 
                double r, double g, double b, const std::string & baseId = "segment", 
                const bool doLineBetweenLastAndFirtPoints = false,
                int viewport = 0 )
{
    for ( uint64_t count = 1; count < cloudIn->points.size(); count++ )
    {
        std::ostringstream streamForNumber( baseId );

        streamForNumber << count;

        std::string segment = streamForNumber.str();

        viewer->addLine<pcl::PointXYZ>( cloudIn->points[ count ], cloudIn->points[ count - 1], 
                                            r, g, b, segment, viewport );

        viewer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, segment, viewport );                                       
    }

    if ( doLineBetweenLastAndFirtPoints )
    {
        std::ostringstream streamForNumber( baseId );

        streamForNumber << cloudIn->points.size();

        std::string segment = streamForNumber.str();

        viewer->addLine<pcl::PointXYZ>( cloudIn->points.back(), cloudIn->points.front(),
                                            r, g, b, segment, viewport );

        viewer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, segment, viewport );  

    }

}


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


                readSonarFileIntoPointCloud( twoFileNames[ count ], twoLines[ count ], leverArm , boresight, NULL );

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


    std::pair< uint64_t, uint64_t > inBothHulls = hullOverlap.computeHullsAndPointsInBothHulls( line1InBothHulls, 
                                                                                        line2InBothHulls, false );    

    std::cout << "Nb points line1 in both hulls: " << inBothHulls.first
        << "\nNb points line2 in both hulls: " << inBothHulls.second << "\n" << std::endl;


    std::chrono::high_resolution_clock::time_point tEnd = std::chrono::high_resolution_clock::now();
    cout << "\n\nTotal time: " << std::chrono::duration_cast<std::chrono::seconds>(tEnd - tStart).count() << "s" << endl;       

    // ------------------------------------ Visualization -----------------------------------------

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters();


    // Viewport for line 1 and 2 (in the x, y, z coordinate system they were in the files)
    int viewport0 = 0;
    viewer->createViewPort( 0.0, 0.0, 0.33, 1.0, viewport0 );    
	viewer->setBackgroundColor ( 1.0, 1.0, 1.0, viewport0 );		

    // Display line 1
    viewer->addPointCloud<pcl::PointXYZ> ( line1, "line1", viewport0 );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "line1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "line1");    

    // Display line 2
    viewer->addPointCloud<pcl::PointXYZ> ( line2, "line2", viewport0 );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "line2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "line2");



    // Viewport for line 1 and 2 in the projection plane
    int viewport1 = 0;
    viewer->createViewPort( 0.33, 0.0, 0.66, 1.0, viewport1 );    
	viewer->setBackgroundColor ( 1.0, 1.0, 1.0, viewport1 );		

    // Get a pointer to points of line 1 in the projection plane
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr line1InPlane = hullOverlap.getConstPtrLineInPlane( true );

    // Display points of line 1 projected in the projection plane
    viewer->addPointCloud<pcl::PointXYZ> ( line1InPlane, "line1InPlane", viewport1 );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "line1InPlane");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "line1InPlane");    

    // Display the hull of line 1 (in the projection plane)
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull1VerticesInPlane (new pcl::PointCloud<pcl::PointXYZ>);

    // Get a pointer to the indices of points of line 1 forming a concave hull in the projection plane
    const std::vector< int > * hull1PointIndices = hullOverlap.getConstPtrVerticesIndices( true );

    for ( uint64_t count = 0; count < hull1PointIndices->size(); count++ )
    {
        hull1VerticesInPlane->push_back( line1InPlane->points[ hull1PointIndices->at( count ) ] );
    }



    drawLinesFromPointToPoint( viewer, hull1VerticesInPlane, 0, 1.0, 0, "segmentHull1", true, viewport1 );

    viewer->addPointCloud<pcl::PointXYZ> ( hull1VerticesInPlane, "hull1VerticesInPlane", viewport1 );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "hull1VerticesInPlane");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "hull1VerticesInPlane"); 



    // Plot first vertice of hull 1
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull1Vertice1InPlane (new pcl::PointCloud<pcl::PointXYZ>);
    hull1Vertice1InPlane->push_back( hull1VerticesInPlane->points.front() );

    viewer->addPointCloud<pcl::PointXYZ> ( hull1Vertice1InPlane, "hull1Vertice1InPlane", viewport1 );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "hull1Vertice1InPlane");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "hull1Vertice1InPlane"); 

    // Plot last vertice of hull 1
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull1VerticeLastInPlane (new pcl::PointCloud<pcl::PointXYZ>);
    hull1VerticeLastInPlane->push_back( hull1VerticesInPlane->points.back() );

    viewer->addPointCloud<pcl::PointXYZ> ( hull1VerticeLastInPlane, "hull1VerticeLastInPlane", viewport1 );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "hull1VerticeLastInPlane");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "hull1VerticeLastInPlane"); 


    // Get a pointer to points of line 2 in the projection plane
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr line2InPlane = hullOverlap.getConstPtrLineInPlane( false );

    // Display points of line 2 projected in the projection plane
    viewer->addPointCloud<pcl::PointXYZ> ( line2InPlane, "line2InPlane", viewport1 );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "line2InPlane");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "line2InPlane");

    // Display the hull of line 2 (in the projection plane)
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull2VerticesInPlane (new pcl::PointCloud<pcl::PointXYZ>);

    // Get a pointer to the indices of points of line 2 forming a concave hull in the projection plane
    const std::vector< int > * hull2PointIndices = hullOverlap.getConstPtrVerticesIndices( false );

    for ( uint64_t count = 0; count < hull2PointIndices->size(); count++ )
    {
        hull2VerticesInPlane->push_back( line2InPlane->points[ hull2PointIndices->at( count ) ] );
    }


    drawLinesFromPointToPoint( viewer, hull2VerticesInPlane, 1.0, 0.5, 0, "segmentHull2", true, viewport1 );


    viewer->addPointCloud<pcl::PointXYZ> ( hull2VerticesInPlane, "hull2VerticesInPlane", viewport1 );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.5, 0, "hull2VerticesInPlane");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "hull2VerticesInPlane"); 


    // Plot first vertice of hull 2
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull2Vertice1InPlane (new pcl::PointCloud<pcl::PointXYZ>);
    hull2Vertice1InPlane->push_back( hull2VerticesInPlane->points.front() );

    viewer->addPointCloud<pcl::PointXYZ> ( hull2Vertice1InPlane, "hull2Vertice1InPlane", viewport1 );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.5, 0, "hull2Vertice1InPlane");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "hull2Vertice1InPlane"); 

    // Plot last vertice of hull 2
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull2VerticeLastInPlane (new pcl::PointCloud<pcl::PointXYZ>);
    hull2VerticeLastInPlane->push_back( hull2VerticesInPlane->points.back() );

    viewer->addPointCloud<pcl::PointXYZ> ( hull2VerticeLastInPlane, "hull2VerticeLastInPlane", viewport1 );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.5, 0, "hull2VerticeLastInPlane");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "hull2VerticeLastInPlane"); 



    // Viewport for the overlap (in the projection plane)
    int viewport2 = 0;
    viewer->createViewPort(0.66, 0.0, 1.0, 1.0, viewport2 );
	viewer->setBackgroundColor ( 1.0, 1.0, 1.0, viewport2 );	


    pcl::PointCloud<pcl::PointXYZ>::Ptr line1InPlaneInBothHull (new pcl::PointCloud<pcl::PointXYZ>);

    const std::vector< uint64_t> * line1InBothHullPointIndices = hullOverlap.getConstPtrlineInBothHullPointIndices( true );

    for ( uint64_t count = 0; count < line1InBothHullPointIndices->size(); count++ )
    {
        line1InPlaneInBothHull->push_back( line1InPlane->points[ line1InBothHullPointIndices->at( count ) ] );
    }

    viewer->addPointCloud<pcl::PointXYZ> ( line1InPlaneInBothHull, "line1InPlaneInBothHull", viewport2 );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "line1InPlaneInBothHull");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "line1InPlaneInBothHull");


    pcl::PointCloud<pcl::PointXYZ>::Ptr line2InPlaneInBothHull (new pcl::PointCloud<pcl::PointXYZ>);

    const std::vector< uint64_t> * line2InBothHullPointIndices = hullOverlap.getConstPtrlineInBothHullPointIndices( false );

    for ( uint64_t count = 0; count < line2InBothHullPointIndices->size(); count++ )
    {
        line2InPlaneInBothHull->push_back( line2InPlane->points[ line2InBothHullPointIndices->at( count ) ] );
    }

    viewer->addPointCloud<pcl::PointXYZ> ( line2InPlaneInBothHull, "line2InPlaneInBothHull", viewport2 );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "line2InPlaneInBothHull");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "line2InPlaneInBothHull");



	// viewer->setPosition( 300, 100 ); // Position of the window on the screen
	// viewer->setSize( 1800, 1200 );		// Size of the window on the screen
	
	viewer->resetCameraViewpoint("Everything");

	// viewer->addCoordinateSystem( 10, 0, 0, 0 );

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



    return 0;
}


