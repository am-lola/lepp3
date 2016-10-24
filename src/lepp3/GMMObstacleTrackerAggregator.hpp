#ifndef LEPP3_GMM_OBSTACLE_TRACKER_AGGREGATOR_H__
#define LEPP3_GMM_OBSTACLE_TRACKER_AGGREGATOR_H__

#include "lepp3/obstacles_new/ObstacleTrackerState.hpp"

namespace lepp
{

/**
 * An interface that all classes that wish to be notified of the result generated
 * by GMMObstacleTracker need to implement.
 */
template<class PointT>
class GMMObstacleTrackerAggregator {
public:
  virtual void updateObstacleTrackingData(
    ar::PointCloudData const& cloud_data,
    VoxelGrid const& vg,
    GMM::RuntimeStat runtime_stats) = 0;
};

} // namespace lepp
#endif // LEPP3_GMM_OBSTACLE_TRACKER_AGGREGATOR_H__
