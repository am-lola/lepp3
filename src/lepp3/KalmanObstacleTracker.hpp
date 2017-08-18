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

class KalmanObstacleTracker : public FrameDataObserver, public FrameDataSubject
{
public:
  /**
   * Creates a new `KalmanObstacleTracker`.
   */
  KalmanObstacleTracker();

  /**
   * The member function that all concrete aggregators need to implement in
   * order to be able to process newly detected obstacles.
   */
  virtual void updateFrame(FrameDataPtr frameData);
private:
  std::map<int, ObstacleKalmanFilter> states_; // map obstacle id to kalman state
  Timer frameTimer_; // used to track time between frames

  // kalman filter parameters
  float noise_position    = 0.01f;
  float noise_velocity    = 0.15f;
  float noise_measurement = 0.10f;
};

KalmanObstacleTracker::KalmanObstacleTracker()
    {}


void KalmanObstacleTracker::updateFrame(FrameDataPtr frameData)
{
  double dt; // time since previous updateFrame call
             /// NOTE: Accuracy may depend on what's going on further up the pipeline.
             ///       It may be worth looking into adding this to FrameData.
  if (frameData->frameNum == 0)
  {
      dt = 1/30.0f; // assume 30hz for first frame
  }
  else
  {
    frameTimer_.stop();
    dt = frameTimer_.duration() / 1000.0f;
  }
  frameTimer_.start();

  // only process frameData if there are obstacles availble
  if (frameData->cloudMinusSurfaces->size() != 0)
  {
    for (auto& obstacle : frameData->obstacles)
    {
      std::cout << "--+ Kalman processing object: " << obstacle->id() << std::endl;
      if (obstacle->id() != 0) // obstacles with id==0 have not been approximated, may not be real
      {
        auto state = states_.find(obstacle->id());
        if (state == states_.end()) // new obstacle
        {
          Eigen::Vector3f pos = Eigen::Vector3f(obstacle->center_point().x,
                                                obstacle->center_point().y,
                                                obstacle->center_point().z);
          ObstacleFilter::State initialState(pos, Eigen::Vector3f::Zero()); // init w/ current position & zero velocity
          ObstacleKalmanFilter kf;
          kf.init(initialState);
          states_[obstacle->id()] = kf;
        }
        else // existing obstacle
        {
          // get obstacle's current position
          Eigen::Vector3f pos = Eigen::Vector3f(obstacle->center_point().x,
                                                obstacle->center_point().y,
                                                obstacle->center_point().z);
          ObstacleFilter::SystemModel sys(noise_position, noise_velocity, dt);
          ObstacleFilter::MeasurementModel mm(noise_measurement);
          state->second.predict(sys);
          auto newState = state->second.update(mm, ObstacleFilter::Measurement(pos));

          /// TODO: Set obstacle position; should probably use a ModelVisitor
          obstacle->set_velocity(Coordinate(newState.velocity().x(),
                                            newState.velocity().y(),
                                            newState.velocity().z()
                                            ));
          std::cout << "--+ Obstacle " << obstacle->id() << " with velocity: " <<
                       obstacle->velocity().x << ", " <<
                       obstacle->velocity().y << ", " <<
                       obstacle->velocity().z << std::endl;
        }
      }
    }
  }

  notifyObservers(frameData);
}

}  // namespace lepp

#endif // LEPP3_KALMAN_OBSTACLE_TRACKER_H__
