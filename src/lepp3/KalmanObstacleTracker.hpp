#ifndef LEPP3_KALMAN_OBSTACLE_TRACKER_H__
#define LEPP3_KALMAN_OBSTACLE_TRACKER_H__

#include <vector>
#include <list>
#include <map>

#include "lepp3/ObstacleFilter.hpp"
#include "lepp3/FrameData.hpp"

#include "lepp3/util/Timer.hpp"

#include "deps/easylogging++.h"

namespace lepp {

class KalmanObstacleTracker
{
public:
  /**
   * Creates a new `KalmanObstacleTracker`.
   */
  KalmanObstacleTracker(float noise_position,
                        float noise_velocity,
                        float noise_measurement)
      : noise_position(noise_position),
        noise_velocity(noise_velocity),
        noise_measurement(noise_measurement)
    {}

  /**
   * Runs an update pass on all obstacles in the given vector (or inits filters
   * for them if they're new) and updates their positions & velocities with
   * filtered values.
   */
  void update(std::vector<lepp::ObjectModelParams>& obstacles);

  /**
   * Discards any current tracking data for the given object
   */
  void reset(int id);

private:
  std::map<int, ObstacleKalmanFilter> states_; // map obstacle id to kalman state
  HiResTimer frameTimer_; // used to track time between frames

  // kalman filter parameters
  float noise_position    = 0.01f;
  float noise_velocity    = 0.15f;
  float noise_measurement = 0.10f;
};

}  // namespace lepp

#endif // LEPP3_KALMAN_OBSTACLE_TRACKER_H__
