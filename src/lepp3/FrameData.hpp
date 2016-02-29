#ifndef LEPP3_FRAME_DATA_H__
#define LEPP3_FRAME_DATA_H__

#include <vector>
#include "lepp3/models/SurfaceModel.h"
#include "lepp3/models/ObjectModel.h"
#include "lepp3/Typedefs.hpp"

namespace lepp
{

struct FrameData
{
//	FrameData() : cloud(new PointCloudT()), cloudMinusSurfaces(new PointCloudT()) {}
	PointCloudConstPtr						cloud;
	PointCloudPtr							cloudMinusSurfaces;
	std::vector<SurfaceModelPtr>			surfaces;
	std::vector<pcl::ModelCoefficients> 	surfaceCoefficients;
	std::vector<PointCloudPtr> 				hulls;
	std::vector<ObjectModelPtr> 			obstacles;
};

}

#endif
