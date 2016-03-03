#ifndef LEPP3_FRAME_DATA_H__
#define LEPP3_FRAME_DATA_H__

#include <vector>
#include <iostream>
#include "lepp3/models/SurfaceModel.h"
#include "lepp3/models/ObjectModel.h"
#include "lepp3/Typedefs.hpp"

namespace lepp
{

struct FrameData
{
	FrameData(int num) : frameNum(num), cloudMinusSurfaces(new PointCloudT()) 
	{
		std::cout << "Frame " << frameNum << std::endl;
	}
	long 							frameNum;
	PointCloudConstPtr				cloud;
	PointCloudPtr					cloudMinusSurfaces;
	std::vector<SurfaceModelPtr>	surfaces;
	std::vector<ObjectModelPtr> 	obstacles;
	std::vector<PointCloudPtr>		obstacleClouds;
};

}

#endif
