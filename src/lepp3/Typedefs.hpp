#ifndef LEPP3_TYPEDEFS_H__
#define LEPP3_TYPEDEFS_H__

#include <pcl/common/common.h>

namespace lepp
{
	// forward declaration
	struct FrameData;
	struct RGBData;
	class SurfaceModel;
	class ObjectModel;

	// typedefs
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;
	typedef typename PointCloudT::Ptr PointCloudPtr;
	typedef typename PointCloudT::ConstPtr PointCloudConstPtr;
	typedef int model_id_t;
	typedef boost::shared_ptr<FrameData> FrameDataPtr;
	typedef boost::shared_ptr<SurfaceModel> SurfaceModelPtr;
	typedef boost::shared_ptr<ObjectModel> ObjectModelPtr;
	typedef boost::shared_ptr<RGBData> RGBDataPtr;
}

#endif