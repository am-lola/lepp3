#ifndef LOLA_SPLITTERS_H__
#define LOLA_SPLITTERS_H__

#include "lepp3/Typedefs.hpp"
#include "lepp3/obstacles/object_approximator/split/SplitCondition.hpp"
#include "lepp3/models/Coordinate.h"
#include "lola/Robot.h"

#include <pcl/common/pca.h>
#include <pcl/common/common.h>

using namespace lepp;

/**
 * A `SplitCondition` implementation that allows for splits to happen only if
 * the object is close enough to the robot.
 */
class DistanceThresholdSplitCondition : public lepp::SplitCondition {
public:
  /**
   * Creates a new `DistanceThresholdSplitCondition` that will only allow
   * splits for objects that are closer to the robot than a given threshold.
   * The distance threshold is in **centimeters**.
   */
  DistanceThresholdSplitCondition(
      int distance,
      Robot& robot)
        : thresh_squared_(distance * distance),
          robot_(robot) {}
  bool shouldSplit(
      int split_depth,
      const PointCloudConstPtr& point_cloud);
private:
  /**
   * The square of the distance threshold at which we will stop splitting
   * objects, i.e. only objects closer than the threshold will be split.
   * The distance itself is given in [cm] (which implies that this is [cm^2]).
   */
  int const thresh_squared_;
  /**
   * A reference to the robot -- required so that we can get an up-to-date
   * position of the robot whenever we need to estimate how far an object is
   * from it.
   */
  Robot const& robot_;
};

bool DistanceThresholdSplitCondition::shouldSplit(
    int split_depth,
    const PointCloudConstPtr& point_cloud) {
  // The distance should be in [cm] so we need to scale up the original points
  // (as they are in [m])
  Coordinate const robot_position = 100 * robot_.robot_position();
  // Compute the centroid of the pointcloud -> approx position of the object
  Eigen::Vector4d centroid_4d;
  pcl::compute3DCentroid(*point_cloud, centroid_4d);
  Coordinate const centroid(
      100 * centroid_4d[0], 100 * centroid_4d[1], 100 * centroid_4d[2]);
  // Now find he distance between the robot's location and the centroid of the
  // cloud, giving an estimate of how far the robot is from the object.
  int const dist = (robot_position - centroid).square_norm();

  // Split only if the object is close enough...
  return dist < thresh_squared_;
}

#endif
