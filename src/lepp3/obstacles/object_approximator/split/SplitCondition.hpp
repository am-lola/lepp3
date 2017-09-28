#ifndef LEPP3_OBJECT_APPROXIMATOR_SPLIT_SPLIT_CONDITION_H__
#define LEPP3_OBJECT_APPROXIMATOR_SPLIT_SPLIT_CONDITION_H__

#include "lepp3/Typedefs.hpp"

namespace lepp {

/**
 * An ABC for classes that provide the functionality of checking whether a
 * point cloud should be split or not.
 */
class SplitCondition {
public:
  /**
   * A pure virtual method that decides whether the given point cloud should be
   * split or not.
   *
   * :param split_depth: The current split depth, i.e. the number of times the
   *     original cloud has already been split
   * :param point_cloud: The current point cloud that should be split by the
   *    `SplitStrategy` implementation.
   * :returns: A boolean indicating whether the cloud should be split or not.
   */
  virtual bool shouldSplit(
      int split_depth,
      const PointCloudConstPtr& point_cloud) = 0;
};

}

#endif
