#ifndef LEPP3_TYPEDEFS_H__
#define LEPP3_TYPEDEFS_H__

namespace lepp
{
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;
	typedef typename PointCloudT::Ptr PointCloudPtr;
	typedef typename PointCloudT::ConstPtr PointCloundConstPtr;
}

#endif
