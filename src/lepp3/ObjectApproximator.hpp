#ifndef LEPP3_OBJECT_APPROXIMATOR_H__
#define LEPP3_OBJECT_APPROXIMATOR_H__

#include "lepp3/Typedefs.hpp"
#include <pcl/common/projection_matrix.h>

namespace lepp {
  /**
   * Forward declaration of the CompositeModel class that represents an
   * approximation.
   */
  class CompositeModel;

/**
 * An abstract base class that classes that wish to generate approximations
 * for point clouds need to implement.
 */
template<class PointT>
class ObjectApproximator {
public:
  /**
   * Generate the approximations for the given point cloud.
   * The method assumes that the given point cloud segment is a single physical
   * object and tries to find the best approximations for this object, using its
   * own specific approximation method.
   */
  virtual boost::shared_ptr<CompositeModel> approximate(
      const PointCloudConstPtr& point_cloud) = 0;
};

}  // namespace lepp

#endif
