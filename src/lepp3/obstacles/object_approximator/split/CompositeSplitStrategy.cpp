#include "CompositeSplitStrategy.hpp"

bool lepp::CompositeSplitStrategy::shouldSplit(int split_depth, const PointCloudConstPtr& point_cloud) {
  size_t const sz = conditions_.size();
  if (sz == 0) {
    // If there are no conditions, do not split the object, in order to avoid
    // perpetually splitting it, since there's no condition that could
    // possibly put an end to it.
    return false;
  }

  for (size_t i = 0; i < sz; ++i) {
    if (!conditions_[i]->shouldSplit(split_depth, point_cloud)) {
      // No split can happen if any of the conditions disallows it.
      return false;
    }
  }

  // Split only if all of the conditions allowed us to split
  return true;
}
