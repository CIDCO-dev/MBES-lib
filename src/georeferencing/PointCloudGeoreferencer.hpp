/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/


#ifndef POINTCLOUDGEOREFERENCER_HPP
#define POINTCLOUDGEOREFERENCER_HPP


#include <cstdint>

#include <pcl/common/common_headers.h>

#include "DatagramGeoreferencer.hpp"
#include "../datagrams/DatagramParserFactory.hpp"

#include "../svp/SvpSelectionStrategy.hpp"



/*!
* \brief PCL point cloud georeferencer class
* \author Guillaume Labbe-Morissette, Christian Bouchard
*
* Extends from DatagramGeoreferencer
*/


class PointCloudGeoreferencer : public DatagramGeoreferencer{

public:
	/**
	* Creates a PointCloudGeoreferencer
	* Georeferences a point cloud
	*
	* @param cloud Point cloud
	* @param georef The georeferencer
	*/
	PointCloudGeoreferencer( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Georeferencing & georef,
							SvpSelectionStrategy & svpStrat )
	: cloud( cloud ),DatagramGeoreferencer(georef, svpStrat) {

	}

	/** Destroys a PointCloudGeoreferencer  */
	virtual ~PointCloudGeoreferencer() {}

	/**
	* Inserts georeferenced ping's position at the end of a point cloud
	*
	* @param ping georeferenced Ping
	* @param quality the quality flag
	* @param intensity the intensity flag
	*/
	virtual void processGeoreferencedPing(Eigen::Vector3d & georeferencedPing, uint32_t quality, 
											int32_t intensity, int positionIndex, int attitudeIndex){

		pcl::PointXYZ point;

		point.x = georeferencedPing(0);
		point.y = georeferencedPing(1);
		point.z = georeferencedPing(2);

		cloud->push_back(point);
	}


private:
	/** Point cloud */
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};



#endif
