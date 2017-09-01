#ifndef LEPP3_KALMAN_OBSTACLE_TRACKER_H__
#define LEPP3_KALMAN_OBSTACLE_TRACKER_H__

#include <unordered_set>
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

class KalmanTrackerFilter : public FrameDataSubject, public FrameDataObserver
{
public:
    KalmanTrackerFilter(float noise_position, float noise_velocity, float noise_measurement) :
        tracker_(noise_position, noise_velocity, noise_measurement)
    {}

    virtual void updateFrame(FrameDataPtr frameData)
    {
        std::unordered_set<int> remove_ids = known_ids_;
        for (const auto& obj : frameData->obstacleParams)
        {
          if (known_ids_.count(obj.id) > 0)
              remove_ids.erase(obj.id);
          else
              known_ids_.insert(obj.id);
        }
        // clear any old ids not found in current frame
        for (auto& id : remove_ids)
        {
            tracker_.reset(id); // clear any tracked state for removed ids
            known_ids_.erase(id);
        }

        // update kalman tracking for objects in current frame
        tracker_.update(frameData->obstacleParams);

        // pass results on down the pipeline
        notifyObservers(frameData);
    }

private:
    KalmanObstacleTracker tracker_;
    std::unordered_set<int> known_ids_;
};

}  // namespace lepp

#endif // LEPP3_KALMAN_OBSTACLE_TRACKER_H__
