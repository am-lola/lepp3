#ifndef LEPP3_TYPEDEFS_H__
#define LEPP3_TYPEDEFS_H__

namespace lepp
{
	// forward declaration
	class FrameData;
	class SurfaceModel;

	// typedefs
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;
	typedef typename PointCloudT::Ptr PointCloudPtr;
	typedef typename PointCloudT::ConstPtr PointCloudConstPtr;
	typedef int model_id_t;
	typedef boost::shared_ptr<FrameData> FrameDataPtr;
	typedef boost::shared_ptr<SurfaceModel> SurfaceModelPtr;
}

#endif