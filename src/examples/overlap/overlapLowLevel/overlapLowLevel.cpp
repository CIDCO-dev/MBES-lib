/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

 /*
 * \author Christian Bouchard
 */


#include <iostream>
// #include <fstream>
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




class PointCloudGeoreferencer : public DatagramGeoreferencer{

public:
	/**
	* Creates a PointCloudGeoreferencer
	* Georeferences a point cloud
	*
	* @param cloud Point cloud
	* @param georef The georeferencer
	*/
	PointCloudGeoreferencer( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Georeferencing & georef )
	: cloud( cloud ),DatagramGeoreferencer(georef) {

	}

	/** Destroys a PointCloudGeoreferencer  */
	virtual ~PointCloudGeoreferencer() {}

	/** Returns a point cloud */
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud(){
		cloud->width = (int) cloud->points.size();
		cloud->height = (int) 1;

		return cloud;
	};

	/**
	* Displays a georeferenced ping's info
	*
	* @param georeferencedPing
	* @param quality the quality flag
	* @param intensity the intensity flag
	*/
	virtual void processGeoreferencedPing(Eigen::Vector3d & ping,uint32_t quality,int32_t intensity){

		pcl::PointXYZ point;

		point.x = ping(0);
		point.y = ping(1);
		point.z = ping(2);

		cloud->push_back(point);
	}


private:
	/** Point cloud */
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};




uint64_t readSonarFile( std::string fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut, 
                    Eigen::Vector3d leverArm, Eigen::Matrix3d boresight,
                    SoundVelocityProfile * svpFile )
{

	Georeferencing * georef = new GeoreferencingLGF(); //TODO: allow TRF through CLI

	PointCloudGeoreferencer pointCloudGeoreferencer( cloudOut,*georef );

	DatagramParser * parser = nullptr;

	try
	{

		std::cout << "\nReading sonar file" << std::endl;

		std::ifstream inFile;
		inFile.open( fileName );

		if (inFile)
		{
			parser = DatagramParserFactory::build( fileName, pointCloudGeoreferencer );
		}
		else
		{
			throw new Exception("Unknown extension");
		}

		parser->parse( fileName );

		std::cout << "\nGeoreferencing point cloud" << std::endl;

		//TODO: get SVP from CLI
		pointCloudGeoreferencer.georeference( leverArm , boresight , NULL );

	}
	catch(Exception * error)
	{
		if(parser)
		    delete parser;        

        std::string text = "\nError while parsing file \n\n\"" + fileName + "\":\n\n" + error->getMessage() + ".\n";
        throw new Exception( text );

	}
	catch ( const char * message )
	{
		if(parser)
		    delete parser;

        std::string text = "\nError while parsing file \n\n\"" + fileName + "\":\n\n" + std::string( message ) + ".\n";
        throw new Exception( text );

	}
	catch (...)
	{
		if(parser)
		    delete parser;

        std::string text = "\nError while parsing file \n\n\"" + fileName + "\":\n\nOther exception.\n";
        throw new Exception( text );
	}


    return cloudOut->points.size();
    
}











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



// ./TestPCLConcaveHullOverlap "/home/christian/Documents/PSR-964 Erreur de celerite/CodeScilabModByCB/SorelErreurCelerite/Line1.txt" "/home/christian/Documents/PSR-964 Erreur de celerite/CodeScilabModByCB/SorelErreurCelerite/Line2.txt"  0.0 0.0 1.0 0.0     1.0 1.0

// ./TestPCLConcaveHullOverlap "../../data/Line1.txt" "../../data/Line2.txt"  0.0 0.0 1.0 0.0     1.0 1.0

// ./TestPCLConcaveHullOverlap "../../data/Asoundings1Observed.txt" "../../data/Asoundings2Observed.txt"  0.0 0.0 1.0 0.0     5.0 5.0

// ./TestPCLConcaveHullOverlap "../../data/Bsoundings1Observed.txt" "../../data/Bsoundings2Observed.txt"  0.0 0.0 1.0 0.0     5.0 5.0

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

    double a = 0;
    double b = 0;
    double c = 1;
    double d = 0;


    convertStringToDouble( argv[ 3 ], a, "Plane parameter 'a'" );
    convertStringToDouble( argv[ 4 ], b, "Plane parameter 'b'" );
    convertStringToDouble( argv[ 5 ], c, "Plane parameter 'c'" );
    convertStringToDouble( argv[ 6 ], d, "Plane parameter 'd'" );

    std::cout << "\na: " << a << "\n" 
        << "b: " << b << "\n"
        << "c: " << c << "\n"
        << "d: " << d << "\n" << std::endl;
    


    double alphaLine1 = 1.0;
    double alphaLine2 = 1.0;

    if ( argc == 9 )
    {
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


        // If ends in .txt: point cloud X, Y, Z
        if (  ends_with( fileNameLine1.c_str(),".txt") )
        {
            readPointCloud( twoFileNames[ count ], twoLines[ count ] );
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
                
                readSonarFile( twoFileNames[ count ], twoLines[ count ], leverArm , boresight, NULL );


            }
            catch(Exception * error)
            {
                cout << error->getMessage();
            }


        }



    }






    // // std::cout << "\nReading both lines from file\n" << std::endl; 

    // // const uint64_t nbPointsLine1 = readPointCloud( fileNameLine1, line1 );

    // // std::cout << "nbPointsLine1: " << nbPointsLine1 << "\n\n" << std::endl;


    // // const uint64_t nbPointsLine2 = readPointCloud( fileNameLine2, line2 );

    // // std::cout << "nbPointsLine2: " << nbPointsLine2 << "\n\n" << std::endl;




    pcl::PointCloud<pcl::PointXYZ>::Ptr line1InBothHulls (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr line2InBothHulls (new pcl::PointCloud<pcl::PointXYZ>);


    std::cout << "Processing to find the overlap\n" << std::endl;

    HullOverlap hullOverlap( line1, line2, a, b, c, d, alphaLine1, alphaLine2 );

    // std::string junk1;
    // std::cout << "\nBefore computeHull. Enter something to continue: ";
    // std::cin >> junk1;


    // std::pair< uint64_t, uint64_t > inBothHulls = hullOverlap.computeHullsAndPointsInBothHulls( nullptr, nullptr, true );
    std::pair< uint64_t, uint64_t > inBothHulls = hullOverlap.computeHullsAndPointsInBothHulls( line1InBothHulls, 
                                                                                        line2InBothHulls, false );    



    std::cout << "Nb points line1 in both hulls: " << inBothHulls.first
        << "\nNb points line2 in both hulls: " << inBothHulls.second << "\n" << std::endl;



    std::chrono::high_resolution_clock::time_point tEnd = std::chrono::high_resolution_clock::now();
    cout << "\n\nTotal time: " << std::chrono::duration_cast<std::chrono::seconds>(tEnd - tStart).count() << "s" << endl;       


    // std::string junk2;
    // std::cout << "\nAfter computeHull. Enter something to continue: ";
    // std::cin >> junk2;


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

    std::cout << "Hull 1 index first point: " << hull1PointIndices->front()
        << "\nHull 1 index last point: " << hull1PointIndices->back() << "\n" << std::endl;
    std::cout << "Hull 1 first point: " << hull1VerticesInPlane->points.front()
        << "\nHull 1 last point: " << hull1VerticesInPlane->points.back()<< "\n" << std::endl;


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

    std::cout << "Hull 2 index first point: " << hull2PointIndices->front()
        << "\nHull 2 index last point: " << hull2PointIndices->back() << "\n" << std::endl;
    std::cout << "Hull 2 first point: " << hull2VerticesInPlane->points.front()
        << "\nHull 2 last point: " << hull2VerticesInPlane->points.back() << "\n" << std::endl;

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


