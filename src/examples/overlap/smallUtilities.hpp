#ifndef SMALLUTILITIES
#define SMALLUTILITIES

#include <iostream>
#include <fstream>

#include <sstream>

#include <cstdint>




#include <pcl/common/common_headers.h>


uint64_t readPointCloud( const std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const uint64_t lastLineToInclude = UINT64_MAX )
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
                        const std::string whatIsIt, const bool needLargerThanZero = false )
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
        if ( needLargerThanZero && numberOut <=0 )
        {
            std::cout << "\n\n" << whatIsIt << ": " << numberOut << " is invalid, need a double larger than 0\n\n" << std::endl;
            exit( 41 );                            
        }                        
    }

}





#endif