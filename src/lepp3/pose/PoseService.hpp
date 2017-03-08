#ifndef LEPP_POSE_POSE_SERVICE_H__
#define LEPP_POSE_POSE_SERVICE_H__

#include <memory>
#include <vector>

#include "lepp3/models/Coordinate.h"
#include "lepp3/models/LolaKinematics.h"

struct HR_Pose_Red;

namespace lepp {

/**
 * This class provides an API for other components to get the current pose information,
 * independent of where this information comes from.
 */
class PoseService {
public:
  /**
   * Virtual destructor because this is an abstract class
   */
  virtual ~PoseService() = default;

  /**
   * Starts the `PoseService`.
   */
  virtual void start() {}

  /**
   * Returns the "World" origin in ODO coordinate system.
   */
  static lepp::Coordinate getRobotPosition(
      const lepp::LolaKinematicsParams& params);

  /**
   * Returns the parameters relevant for coordinate system
   * transformations, extracted from the currently known robot pose.
   */
  lepp::LolaKinematicsParams getParams() const;

  /**
   * Force a dispatch of the next frame
   */
  virtual void triggerNextFrame() = 0;

private:
  /**
   * Obtains the current pose information. Using this method is completely
   * thread safe.
   */
  virtual std::shared_ptr<HR_Pose_Red> getCurrentPose() const = 0;

};

}

#endif
