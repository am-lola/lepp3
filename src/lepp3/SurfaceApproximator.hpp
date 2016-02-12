#ifndef lepp3_SURFACE_APPROXIMATOR_H__
#define lepp3_SURFACE_APPROXIMATOR_H__

#include <pcl/common/projection_matrix.h>

namespace lepp {
  /**
   * Forward declaration of the PlaneModel class that represents an
   * approximation.
   */
  class PlaneModel;

/**
 * An abstract base class that classes that wish to generate approximations
 * for point clouds need to implement.
 */
template<class PointT>
class SurfaceApproximator {
public:
  /**
   * Generate the approximations for the given point cloud.
   * The method assumes that the given point cloud segment is a single physical
   * object and tries to find the best approximations for this surface, using its
   * own specific border approximation method.
   */
  virtual boost::shared_ptr<PlaneModel> approximate(
      PointCloudConstPtr &point_cloud, pcl::ModelCoefficients &coeffs) = 0;
};

}  // namespace lepp

#endif
