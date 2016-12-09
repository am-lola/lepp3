#ifndef LOLA_POSE_SERVICE_H__
#define LOLA_POSE_SERVICE_H__

#include <iface_vis.h>
#include <vector>

#include "lola/PoseObserver.hpp"
#include "lepp3/models/Coordinate.h"

/**
 * This class provides an API for other components to get the current pose information,
 * independent of where this information comes from.
 */
class PoseService {
public:
  /**
   * Creates a Pose service which listens on a UDP port
   */
  static boost::shared_ptr<PoseService> FromUdp(std::string const& host, int port);
  /**
   * Creates a Pose service from a saved file
   */
  static boost::shared_ptr<PoseService> FromFile(std::string const& filename);

  /**
   * Virtual destructor because this is an abstract class
   */
  virtual ~PoseService() = default;

  /**
   * Starts the `PoseService`.
   */
  virtual void start() {};

  /**
   * Obtains the current pose information. Using this method is completely
   * thread safe.
   */
  virtual HR_Pose_Red getCurrentPose() const = 0;

  /**
   * Returns the "World" origin in ODO coordinate system.
   */
  lepp::Coordinate getRobotPosition() const;

  /**
   * Returns the parameters relevant for coordinate system
   * transformations, extracted from the currently known robot pose.
   */
  LolaKinematicsParams getParams() const;

  /**
   * Attaches a new TFObserver to the pose service.
   * Each observer will get notified once the PoseService has received a new
   * pose and converted it to LolaKinematicsParams.
   */
  void attachObserver(boost::shared_ptr<TFObserver> observer);

  /**
    * public helper method.  Notifies all known observers that a new pose
    * has been received.
    */
  void notifyObservers(int idx, LolaKinematicsParams& params);

  /**
   * Force a dispatch of the next frame
   */
  virtual void triggerNextFrame() {};

private:
  std::vector<boost::shared_ptr<TFObserver>> observers_;
};

#endif
