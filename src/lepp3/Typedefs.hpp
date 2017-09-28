#ifndef LEPP3_TYPEDEFS_H__
#define LEPP3_TYPEDEFS_H__

#include <pcl/common/common.h>
#include <vector>
#include <memory>

namespace lepp
{
	// forward declaration
	struct FrameData;
	struct RGBData;
	struct SurfaceData;
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
	typedef boost::shared_ptr<SurfaceData> SurfaceDataPtr;
	typedef unsigned int mesh_handle_t;


  template <typename T>
  using DefaultDelete = typename std::default_delete<T>;
  template <typename T, typename TD = DefaultDelete<T>>
  using UniquePtr = std::unique_ptr<T, TD>;
  template <typename T>
  using SharedPtr = boost::shared_ptr<T>;
  template <typename T>
  using Vector = std::vector<T>;
}

#endif