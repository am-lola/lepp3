#ifndef LEPP3_BASE_SEGMENTER_H__
#define LEPP3_BASE_SEGMENTER_H__

#include <vector>

#include "lepp3/Typedefs.hpp"
#include <pcl/common/projection_matrix.h>

namespace lepp {

/**
 * An abstract base class for classes that wish to provide point cloud
 * segmentation functionality.
 *
 * Concrete implementations should implement the pure virtual method returning
 * a vector of pcl::PointClouds representing segments in the original point
 * cloud.
 */
template<class PointT>
class BaseSegmenter {
public:
  /**
   * Returns a number of point clouds representing segments from the given
   * original point cloud.
   */
  virtual void segment(
      const PointCloundConstPtr& cloud,
      std::vector<PointCloundConstPtr> &surfaces,
      PointCloudPtr &cloudMinusSurfaces) = 0;
};

} // namespace lepp

#endif
