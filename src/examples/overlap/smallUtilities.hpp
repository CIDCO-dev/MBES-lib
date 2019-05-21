/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

 /*
 * \author Christian Bouchard
 */


#ifndef SMALLUTILITIES
#define SMALLUTILITIES

#include <iostream>
#include <fstream>

#include <sstream>

#include <cstdint>

#include <pcl/common/common_headers.h>


#include "../../DatagramGeoreferencer.hpp"
#include "../../datagrams/DatagramParserFactory.hpp"

#include "../../svp/SoundVelocityProfile.hpp"

#include "../../math/Boresight.hpp"

#include "../../utils/StringUtils.hpp"
#include "../../utils/Exception.hpp"



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

	/**
	* Inserts georeferenced ping's position at the end of a point cloud
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




uint64_t readSonarFileIntoPointCloud( std::string fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut, 
                    Eigen::Vector3d leverArm, Eigen::Matrix3d boresight,
                    SoundVelocityProfile * svpFile )
{

	// Georeferencing * georef = new GeoreferencingLGF(); //TODO: allow TRF through CLI
    Georeferencing * georef = new GeoreferencingTRF(); //TODO: allow TRF through CLI

	PointCloudGeoreferencer pointCloudGeoreferencer( cloudOut,*georef );

	DatagramParser * parser = nullptr;

	try
	{

		std::cout << "\n\nReading sonar file \"" << fileName << "\"" << std::endl;

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

		
		pointCloudGeoreferencer.georeference( leverArm , boresight , svpFile );

	}
	catch ( Exception * error )
	{
		if ( parser )
		    delete parser;        

        std::string text = "\nError while parsing file \n\n\"" + fileName + "\":\n\n" + error->getMessage() + ".\n";
        throw new Exception( text );

	}
	catch ( const char * message )
	{
		if ( parser )
		    delete parser;

        std::string text = "\nError while parsing file \n\n\"" + fileName + "\":\n\n" + std::string( message ) + ".\n";
        throw new Exception( text );

	}
	catch (...)
	{
		if ( parser )
		    delete parser;

        std::string text = "\nError while parsing file \n\n\"" + fileName + "\":\n\nOther exception.\n";
        throw new Exception( text );
	}


    return cloudOut->points.size();
    
}




uint64_t readTextFileIntoPointCloud( const std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const uint64_t lastLineToInclude = UINT64_MAX )
{   
    pcl::PointXYZ point;

    int nbColumns = 3;

    double dataRead[ 3 ];
    
    uint64_t countLines = 0;

    std::ifstream myfile( filename );

    if ( myfile.is_open() )
    {					
        std::string line;	

        // Read the title line
        getline( myfile, line );
                       
        while ( getline( myfile, line ) && countLines < lastLineToInclude )
        {																								
            std::istringstream lineStream( line );
             
            
            for ( int countCol = 0; countCol < nbColumns; countCol++ )
            {
                
                if ( ! ( lineStream >> dataRead[ countCol ] ) )
                {

                    std::cout << "\n\nFile \"" << filename 
                            << "\"\nCould not read element, for line #" 
                            << countLines + 1 << ", col #" << countCol + 1                             
                            << "\n(numbering starts at 1), \n" << std::endl;
                    exit( 11 );                            	
                }	                
                
                
            } 

            point.x = dataRead[ 0 ];
            point.y = dataRead[ 1 ];

            // point.z = 0; // dataRead[ 2 ];
            point.z = dataRead[ 2 ];

            cloud->push_back( point );

            countLines++;
						
        }                                

        myfile.close();
    }
    else
    {
        std::cout << "\n\nCould not open file \"" << filename << "\"\n" << std::endl;
        exit( 22 );	
    }

    return countLines;
}





void convertStringToDouble( std::string textIn, double & numberOut, 
                        const std::string whatIsIt, const bool needAboveZero = false )
{
    std::istringstream text( textIn ); 

    text >> numberOut;        
    if ( text.fail() )
    {
        std::cout << "\n\n" << whatIsIt << ": Could not convert \"" << textIn << "\" into a double\n\n" << std::endl;
        exit( 40 );
    }
    else
    {
        if ( needAboveZero && numberOut <=0 )
        {
            std::cout << "\n\n" << whatIsIt << ": " << numberOut << " is invalid, need a double bigger than 0\n\n" << std::endl;
            exit( 41 );                            
        }                        
    }

}



#endif