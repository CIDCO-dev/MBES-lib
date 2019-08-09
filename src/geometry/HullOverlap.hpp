/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

 /*
 * \author Christian Bouchard
 */

#ifndef HULLOVERLAP_HPP
#define HULLOVERLAP_HPP

#include <iostream>
#include <cstdint>

#include <vector>

#include <utility>      // std::pair, std::make_pair

#include <pcl/common/common_headers.h>

#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/PointIndices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>


#include <Eigen/Dense>
#include <Eigen/Geometry> // For cross product


class HullOverlap 
{

public:    

	/**
	* Creates a HullOverlap
	*
	* @param line1In Point cloud for line #1
	* @param line2In Point cloud for line #2
    * @param a projection plane coefficient 'a' in ax + by + cz + d = 0
    * @param b projection plane coefficient 'b' in ax + by + cz + d = 0
    * @param c projection plane coefficient 'c' in ax + by + cz + d = 0
    * @param d projection plane coefficient 'd' in ax + by + cz + d = 0
    * @param alpha1 Concave hull computation parameter to use with line #1
    * @param alpha2 Concave hull computation parameter to use with line #2
	*/
    HullOverlap( pcl::PointCloud<pcl::PointXYZ>::ConstPtr line1In, 
                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr line2In,
                    double a, double b, double c, double d,
                    double alphaLine1 = 1.0, double alphaLine2 = 1.0 )
                    :   line1( line1In ), line2( line2In ),                     
                        a( a ), b( b ), c( c ), d( d ),
                        alphaLine1( alphaLine1 ), alphaLine2( alphaLine2 ),

                        coefficients ( new pcl::ModelCoefficients() ),

                        line1InPlane (new pcl::PointCloud<pcl::PointXYZ>),
                        line2InPlane (new pcl::PointCloud<pcl::PointXYZ>),

                        line1InPlane2D (new pcl::PointCloud<pcl::PointXYZ>),
                        line2InPlane2D (new pcl::PointCloud<pcl::PointXYZ>),

                        hull1Vertices (new pcl::PointCloud<pcl::PointXYZ>),
                        hull2Vertices (new pcl::PointCloud<pcl::PointXYZ>),
                    
                       // Initialize to dummy values
                       vector1( 1, 0, 0 ), vector2( 0, 1, 0 ), refPoint( 0.0, 0.0, 0.0 )

    {       
        coefficients->values.resize(4);
        coefficients->values[0] = a;
        coefficients->values[1] = b;
        coefficients->values[2] = c;
        coefficients->values[3] = d;
    }


	/**
	* Returns a pair with the number of points in line #1 and in line #2 that are in the overlap area of the two lines.
	* The points are place in the point clouds pointed to by line1InBothHull and line2InBothHull
    *    
	* @param[out] line1InBothHull Point cloud of points in line #1 in the overlap area of the two lines
	* @param[out] line2InBothHull Point cloud of points in line #2 in the overlap area of the two lines
	*/
    std::pair< uint64_t, uint64_t > computePointsInBothHulls( pcl::PointCloud<pcl::PointXYZ>::Ptr line1InBothHull,
                                                              pcl::PointCloud<pcl::PointXYZ>::Ptr line2InBothHull )
    {
        return computeHullsAndPointsInBothHulls( line1InBothHull, line2InBothHull, true );
    }



	/**
	* Returns a pair with the number of points in line #1 and in line #2 that are in the overlap area of the two lines,
	* the points are place in the point clouds pointed to by line1InBothHull and line2InBothHull
    *    
	* @param[out] line1InBothHull Point cloud of points in line #1 in the overlap area of the two lines
	* @param[out] line2InBothHull Point cloud of points in line #2 in the overlap area of the two lines
    * @param[in] line2InBothHull minimalMemory bool variable, true to specify to try and minimize the memory usage
	*/
    // Put back to public to be able to get indices to the points
    std::pair< uint64_t, uint64_t > computeHullsAndPointsInBothHulls( 
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr line1InBothHull = nullptr,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr line2InBothHull = nullptr, 
                                                const bool minimalMemory = false )
    {
        
        if ( line1InBothHull != nullptr )
            line1InBothHull->clear();

        if ( line2InBothHull != nullptr )
            line2InBothHull->clear();


        std::cout << "\nProjecting line 1 in plane\n" << std::endl;

        // Project line 1 in plane
        createCloudFromProjectionInPlane( line1, line1InPlane );

        std::cout << "line1InPlane->points.size(): " << line1InPlane->points.size() << "\n" << std::endl;        

        computeTwoVectorsAndRefPoint();

        std::cout << "\nExpressing points of line 1 in the projection plane using a 2D coordinate system\n" << std::endl;

        createCloudInPlane2D( line1InPlane, line1InPlane2D );    

        if ( minimalMemory )
        {
            // Delete the dynamically allocated memory
            line1InPlane.reset();
            line1InPlane = nullptr;            
        }

        std::cout << "line1InPlane2D->points.size(): " << line1InPlane2D->points.size() << "\n" << std::endl; 


        std::cout << "\nProjecting line 2 in plane\n" << std::endl;

        // Project line 2 in plane
        createCloudFromProjectionInPlane( line2, line2InPlane );

        std::cout << "line2InPlane->points.size(): " << line2InPlane->points.size() << "\n" << std::endl;

        std::cout << "\nExpressing points of line 2 in the projection plane using a 2D coordinate system\n" << std::endl;

        createCloudInPlane2D( line2InPlane, line2InPlane2D );

        if ( minimalMemory )
        {
            // Delete the dynamically allocated memory
            line2InPlane.reset();
            line2InPlane = nullptr;            
        }


        std::cout << "line2InPlane2D->points.size(): " << line2InPlane2D->points.size() << "\n" << std::endl;


        //http://www.pointclouds.org/documentation/tutorials/hull_2d.php

        std::cout << "\nFinding Hull 1\n" << std::endl;

        // Create a Concave Hull for line 1
        computeVerticesOfConcaveHull( line1InPlane2D, alphaLine1, hull1Vertices, hull1PointIndices, ! minimalMemory );


        std::cout << "Finding Hull 2\n" << std::endl;

        // Create a Concave Hull for line 2
        computeVerticesOfConcaveHull( line2InPlane2D, alphaLine2, hull2Vertices, hull2PointIndices, ! minimalMemory );    

        std::cout << "hull1Vertices->points.size(): " << hull1Vertices->points.size() << "\n" 
            << "hull2Vertices->points.size(): " << hull2Vertices->points.size() << "\n" << std::endl;


        // If the hulls of the lines where found correctly, points of line 1 are within the hull of line 1.
        // So only need to check that a point of line 1 is part of hull 2 to know that it is part of both hulls.
        // Same idea for points of line 2.


        if ( line1InBothHull != nullptr && line2InBothHull != nullptr )
        {
            if ( minimalMemory )
            {
                std::cout << "Finding points of Line 1 inside Hull 2\n\n" << std::endl;
            
                findPointsInHullOnlyPoints( line1, line1InPlane2D, line1InBothHull, hull2Vertices );

                // Delete the dynamically allocated memory
                line1InPlane2D.reset();
                line1InPlane2D = nullptr;    

                hull2Vertices.reset();
                hull2Vertices = nullptr;

                std::cout << "Finding points of Line 2 inside Hull 1\n\n" << std::endl;

                findPointsInHullOnlyPoints( line2, line2InPlane2D, line2InBothHull, hull1Vertices );

                // Delete the dynamically allocated memory
                line2InPlane2D.reset();
                line2InPlane2D = nullptr;    

                hull1Vertices.reset();
                hull1Vertices = nullptr;

                std::cout << "line1InBothHull->points.size(): " << line1InBothHull->points.size() << "\n" 
                    << "line2InBothHull->points.size(): " << line2InBothHull->points.size() << "\n" << std::endl;

                return std::make_pair( line1InBothHull->size(), line2InBothHull->size() );

            }
            else
            {
                
                std::cout << "Finding points of Line 1 inside Hull 2 (and the indices)\n\n" << std::endl;
            
                findPointsInHull( line1, line1InPlane2D, line1InBothHull, line1InBothHullPointIndices, hull2Vertices );


                std::cout << "Finding points of Line 2 inside Hull 1 (and the indices)\n\n" << std::endl;

                findPointsInHull( line2, line2InPlane2D, line2InBothHull, line2InBothHullPointIndices, hull1Vertices );


                std::cout << "line1InBothHull->points.size(): " << line1InBothHull->points.size() << "\n" 
                    << "line2InBothHull->points.size(): " << line2InBothHull->points.size() << "\n" << std::endl;

                return std::make_pair( line1InBothHull->size(), line2InBothHull->size() );


            }
            
            
        }   
        else
        {
            std::cout << "Finding indices of points of Line 1 inside Hull 2\n\n" << std::endl;
        
            findPointsInHullOnlyPointIndices( line1InPlane2D, line1InBothHullPointIndices, hull2Vertices );


            std::cout << "Finding indices of points of Line 2 inside Hull 1\n\n" << std::endl;

            findPointsInHullOnlyPointIndices( line2InPlane2D, line2InBothHullPointIndices, hull1Vertices );


            std::cout << "line1InBothHullPointIndices.size(): " << line1InBothHullPointIndices.size() << "\n" 
                << "line2InBothHullPointIndices.size(): " << line2InBothHullPointIndices.size() << "\n" << std::endl; 

            return std::make_pair( line1InBothHullPointIndices.size(), line2InBothHullPointIndices.size() );
        }


    }


    // Uncommented to get access to the point indices
    const std::vector< uint64_t > * getConstPtrlineInBothHullPointIndices( const bool isLine1 )
    {
        if ( isLine1 )
            return & ( line1InBothHullPointIndices );
        else 
            return & ( line2InBothHullPointIndices );
    }



private:

    // These functions were public when wanted to look at details of projection, etc
    // pcl::PointCloud<pcl::PointXYZ>::ConstPtr getConstPtrLineInPlane( const bool isLine1 )
    // {
    //     if ( isLine1 )
    //         return line1InPlane;
    //     else 
    //         return line2InPlane;
    // }

    // const std::vector< int > * getConstPtrVerticesIndices( const bool isLine1 )
    // {
    //     if ( isLine1 )
    //         return & ( hull1PointIndices.indices );
    //     else 
    //         return & ( hull2PointIndices.indices );
    // }







	/**
	* Computes the projection of a point cloud onto a plane
    *  
    * @param[in] cloudIn Point cloud to project on the plane
    * @param[out] cloudOut Point cloud resulting from the projection
	*/
    void createCloudFromProjectionInPlane( pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudIn,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut )
    {
        cloudOut->clear();
        cloudOut->reserve( cloudIn->points.size() );

        // Create the filtering object
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType( pcl::SACMODEL_PLANE );

        proj.setInputCloud( cloudIn );
        proj.setModelCoefficients( coefficients );

        proj.filter( *cloudOut );
    }


	/**
	* Computes two vectors and sets a reference point used to express point positions on the
    * projection plane using only two dimensions 
	*/
    void computeTwoVectorsAndRefPoint()
    {

        // Two vectors and a reference point to span the projection plane
        // so that points in the projection plane can be expressed in
        // a coordinate system with vector1, vector2, and refPoint.

        refPoint = line1InPlane->points[ 0 ];

        // Vector #1: from first point in line to last point in line, normalized    

        const uint64_t nbPointLine1 = line1InPlane->points.size();


        vector1 << line1InPlane->points[ nbPointLine1 - 1 ].x - line1InPlane->points[ 0 ].x,
                    line1InPlane->points[ nbPointLine1 - 1 ].y - line1InPlane->points[ 0 ].y,
                    line1InPlane->points[ nbPointLine1 - 1 ].z - line1InPlane->points[ 0 ].z;

        std::cout << "vector1 before normalization:\n" << vector1 << "\n\n"; 


        vector1 = vector1 / vector1.norm();

        std::cout << "vector1 after normalization:\n" << vector1 << "\n\n"; 


        Eigen::Vector3d normalToPlane;

        normalToPlane <<  a, b, c;

        // Vector #2: perpendicular to the normal to the plane and to vector #1
        vector2 = normalToPlane.cross( vector1 );

        std::cout << "vector2 before normalization:\n" << vector2 << "\n\n"; 


        vector2 = vector2 / vector2.norm();

        std::cout << "vector2 after normalization:\n" << vector2 << "\n\n";   

        // Sanity check
        std::cout << "vector1 dot vector2: " << vector1.dot( vector2 ) << "    (should be 0)\n\n";
    }


	/**
	* Computes a 2D representation of points on the projection plane
    *  
    * @param[in] cloudIn Point cloud on the projection plane expressed in 3D
    * @param[out] cloudOut Point cloud on the projection plane expressed in 2D
	*/
    void createCloudInPlane2D( pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudIn,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut )
    {
        // Build a point cloud where points in the projection plane are expressed in
        // the coordinate system with vector1, vector2, and refPoint.

        cloudOut->clear();
        cloudOut->reserve( cloudIn->points.size() );

        for ( uint64_t count = 0; count < cloudIn->points.size(); count++ )
        {
            pcl::PointXYZ point;

            // projection along vector 1
            point.x = ( cloudIn->points[ count ].x - refPoint.x ) * vector1( 0 )
                        + ( cloudIn->points[ count ].y - refPoint.y ) * vector1( 1 )
                        + ( cloudIn->points[ count ].z - refPoint.z ) * vector1( 2 );

            // projection along vector 2
            point.y = ( cloudIn->points[ count ].x - refPoint.x ) * vector2( 0 )
                        + ( cloudIn->points[ count ].y - refPoint.y ) * vector2( 1 )
                        + ( cloudIn->points[ count ].z - refPoint.z ) * vector2( 2 );                    

            point.z = 0;

            cloudOut->push_back( point );

        }
    }


	/**
	* Computes the vertices of a concave hull for points on the projection plane
    *  
    * @param[in] cloudIn Point cloud on the projection plane expressed in 2D
    * @param[in] alpha Concave hull computation parameter to use
    * @param[out] hullVertices Computed vertices of the concave hull
    * @param[out] hullPointIndices Indices of the points in cloudIn making up the hull
    * @param[in] keepInformation bool variable, true to specify to put the indices of the points in cloudIn in hullPointIndices
	*/
    void computeVerticesOfConcaveHull( pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudIn,
                                        const double alpha, 
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr hullVertices,
                                        pcl::PointIndices & hullPointIndices, const bool keepInformation = true )
    {

        hullVertices->clear();

        pcl::ConcaveHull<pcl::PointXYZ> concaveHull;

        if ( keepInformation )
            concaveHull.setKeepInformation( true ); // To be able to use function getHullPointIndices()

        concaveHull.setInputCloud( cloudIn );
        concaveHull.setAlpha( alpha );
        concaveHull.reconstruct( * hullVertices );

        if ( keepInformation )
            // Get indices of points making the hull
            concaveHull.getHullPointIndices( hullPointIndices );
    }

	/**
	* Find points that are within a concave hull. 
    * Provides the points and their indices within the original line
    *  
    * @param[in] lineOriginal Point cloud of points on the line
    * @param[in] cloudIn Point cloud on the projection plane expressed in 2D
    * @param[out] cloudOut Point cloud of points on the line that are within the hull
    * @param[out] indexPointInHull Indices of the points on the line that are within the hull
    * @param[in] hullVertices Vertices of the concave hull
	*/
    void findPointsInHull( pcl::PointCloud<pcl::PointXYZ>::ConstPtr lineOriginal,
                                pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudIn, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut,
                                std::vector< uint64_t > & indexPointInHull,
                                pcl::PointCloud<pcl::PointXYZ>::ConstPtr hullVertices )    
    {
        cloudOut->clear();
        indexPointInHull.clear();

        for ( uint64_t count = 0; count < cloudIn->points.size(); count++ )
        {
            if ( pcl::isXYPointIn2DXYPolygon( cloudIn->points[ count ], *hullVertices ) )
            {
                cloudOut->push_back( lineOriginal->points[ count ] );
                indexPointInHull.push_back( count );
            }
        }
    
    }


	/**
	* Find indices of points that are within a concave hull. 
    *  
    * @param[in] cloudIn Point cloud on the projection plane expressed in 2D
    * @param[out] indexPointInHull Indices of the points on the line that are within the hull
    * @param[in] hullVertices Vertices of the concave hull
	*/
    void findPointsInHullOnlyPointIndices( pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudIn, 
                                    std::vector< uint64_t > & indexPointInHull,
                                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr hullVertices )    
    {
        indexPointInHull.clear();

        for ( uint64_t count = 0; count < cloudIn->points.size(); count++ )
        {
            if ( pcl::isXYPointIn2DXYPolygon( cloudIn->points[ count ], *hullVertices ) )
                indexPointInHull.push_back( count );
        }
    
    }

	/**
	* Find points that are within a concave hull. 
    *  
    * @param[in] lineOriginal Point cloud of points on the line
    * @param[in] cloudIn Point cloud on the projection plane expressed in 2D
    * @param[out] cloudOut Point cloud of points on the line that are within the hull
    * @param[in] hullVertices Vertices of the concave hull
	*/
    void findPointsInHullOnlyPoints( pcl::PointCloud<pcl::PointXYZ>::ConstPtr lineOriginal,
                                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudIn, 
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut,
                                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr hullVertices )    
    {
        cloudOut->clear();

        for ( uint64_t count = 0; count < cloudIn->points.size(); count++ )
        {
            if ( pcl::isXYPointIn2DXYPolygon( cloudIn->points[ count ], *hullVertices ) )
                cloudOut->push_back( lineOriginal->points[ count ] );
        }
    
    }




// ----------------------------- Variables ------------------------------------------------

    /**Point cloud for line #1*/
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr line1;

    /**Point cloud for line #2*/
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr line2;

    
    /**Projection plane coefficient 'a' in ax + by + cz + d = 0*/
    const double a;

    /**Projection plane coefficient 'b' in ax + by + cz + d = 0*/
    const double b; 

    /**Projection plane coefficient 'c' in ax + by + cz + d = 0*/
    const double c; 

    /**Projection plane coefficient 'd' in ax + by + cz + d = 0*/
    const double d;


    /**Concave hull computation parameter to use with line #1*/
    double alphaLine1; // Alpha value to compute the concave hull for line #1

    /**Concave hull computation parameter to use with line #2*/
    double alphaLine2; // Alpha value to compute the concave hull for line #2

    /**Coefficients for the plane, ax + by + cz + d = 0 */
    pcl::ModelCoefficients::Ptr coefficients;

    /**Point cloud of the projection of line #1 on the plane, expressed in 3D*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr line1InPlane;

    /**Point cloud of the projection of line #2 on the plane, expressed in 3D*/    
    pcl::PointCloud<pcl::PointXYZ>::Ptr line2InPlane;

    /**Point cloud of the projection of line #1 on the plane, expressed in 2D*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr line1InPlane2D;

    /**Point cloud of the projection of line #2 on the plane, expressed in 2D*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr line2InPlane2D;


    /**Vertices of the concave hull for line #1*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull1Vertices;

    /**Vertices of the concave hull for line #2*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull2Vertices;

    /**Indices of the points in line #1 whose projection on the plane makes up its hull*/
    pcl::PointIndices hull1PointIndices;

    /**Indices of the points in line #2 whose projection on the plane makes up its hull*/
    pcl::PointIndices hull2PointIndices;

    /**Indices of the points in line #1 that are within both hulls*/
    std::vector< uint64_t > line1InBothHullPointIndices;

    /**Indices of the points in line #2 that are within both hulls*/
    std::vector< uint64_t > line2InBothHullPointIndices;       


    /**First computed orthonormal vector used to express points on the projection plane in 2D*/
    Eigen::Vector3d vector1;

    /**Second computed orthonormal vector used to express points on the projection plane in 2D*/
    Eigen::Vector3d vector2;

    /**Referenced point used to express points on the projection plane in 2D*/
    pcl::PointXYZ refPoint;

};

#endif