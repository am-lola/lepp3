#ifndef LEPP3_GMM_OBSTACLE_TRACKER_AGGREGATOR_H__
#define LEPP3_GMM_OBSTACLE_TRACKER_AGGREGATOR_H__

#include "lepp3/GMMObstacleTrackerState.hpp"
#include "lepp3/util/VoxelGrid3D.h"

namespace lepp
{

/**
 * An interface that all classes that wish to be notified of the result generated
 * by GMMObstacleTracker need to implement.
 */
class GMMObstacleTrackerAggregator {
public:
  virtual void updateObstacleTrackingData(
    ar::PointCloudData const& cloud_data,
    lepp::util::VoxelGrid3D const& vg,
    GMM::RuntimeStat runtime_stats) = 0;

  /**
   * Notify any attached observer of each of the obstacle states (according to
   * the kalman filter).
   */
  virtual void updateObstacleState(
    GMM::State& state, GMM::VisModifyFlag const& flag) = 0;
};

} // namespace lepp
#endif // LEPP3_GMM_OBSTACLE_TRACKER_AGGREGATOR_H__
