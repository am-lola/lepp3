#ifndef LEPP3_OBJECT_APPROXIMATOR_SPLIT_SPLIT_CONDITIONS_H__
#define LEPP3_OBJECT_APPROXIMATOR_SPLIT_SPLIT_CONDITIONS_H__

#include "SplitCondition.hpp"

#include <pcl/common/pca.h>

#include "lepp3/models/Coordinate.h"

namespace lepp {

/**
 * A `SplitCondition` that allows the split to be made only if the split depth
 * has not exceeded the given limit.
 */
class DepthLimitSplitCondition : public SplitCondition {
public:
  DepthLimitSplitCondition(int depth_limit) : limit_(depth_limit) {}

  bool shouldSplit(
      int split_depth,
      const PointCloudConstPtr& point_cloud) {
    return split_depth < limit_;
  }

private:
  int const limit_;
};

/**
 * A `SplitCondition` that allows the split to be made only if the object's
 * volume is larger than the given limit.
 */
class SizeLimitSplitCondition : public SplitCondition {
public:
  /**
   * Creates a new `SizeLimitSplitCondition`. The size limit that is given is
   * the minimum volume of the object (in **cubic centimeters**) for which a
   * split will be allowed.
   */
  SizeLimitSplitCondition(int size_limit) : limit_(size_limit) {}

  bool shouldSplit(
      int split_depth,
      const PointCloudConstPtr& point_cloud) {
    // Find the limits of the bounding box of the cloud
    PointT min_pt;
    PointT max_pt;
    pcl::getMinMax3D(*point_cloud, min_pt, max_pt);

    // Calculate the volume of the box bounded by those two points.
    // Make sure the units are centimeters.
    Coordinate const sz = 100 * (Coordinate(max_pt) - Coordinate(min_pt));
    int const volume = static_cast<int>(
        (sz.x * sz.x * sz.x) + (sz.y * sz.y * sz.y) + (sz.z * sz.z * sz.z));

    // Split only if the volume is larger than the limit
    return volume > limit_;
  }

private:
  int const limit_;
};

/**
 * A `SplitCondition` that stops the splitting of objects that are certainly a
 * very regular sphere or a very regular capsule. This is because there is no
 * point in splitting those further, instead of simply approximating them with
 * a single instance of that shape.
 */
class ShapeSplitCondition : public SplitCondition {
public:
  ShapeSplitCondition()
      : sphere1(.7), sphere2(.1),
        capsule(.25) {}

  ShapeSplitCondition(double sphere1, double sphere2, double capsule)
      : sphere1(sphere1), sphere2(sphere2), capsule(capsule) {}

  bool shouldSplit(
      int split_depth,
      const PointCloudConstPtr& point_cloud) {
    float major_value, middle_value, minor_value;
    pcl::PCA<PointT> pca;
    pca.setInputCloud(point_cloud);
    Eigen::Vector3f eigenvalues = pca.getEigenValues();
    major_value = eigenvalues(0);
    middle_value = eigenvalues(1);
    minor_value = eigenvalues(2);

    if ((middle_value / major_value > sphere1) && (minor_value / major_value > sphere2)) {
      // This is very much a sphere, so we don't split it any more.
      return false;
    }

    if (middle_value / major_value < capsule) {
      // This is very much a capsule: stop the splitting.
      return false;
    }

    // The object didn't fall squarely into any of the known categories, so for
    // all it's concerned, the object could be split.
    return true;
  }

private:
  double sphere1;
  double sphere2;
  double capsule;
};

}

#endif
