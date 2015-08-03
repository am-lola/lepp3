#ifndef LOLA_ROBOT_H__
#define LOLA_ROBOT_H__

#include "lola/PoseService.h"
#include "lepp2/models/ObjectModel.h"

/**
 * A facade in front of different services and methods that the robot
 * supports.
 *
 * It wraps a number of services and provides a unified interface over
 * their methods. Additionally, it provides some robot-specific functionality
 * based on the information provided by the services.
 */
class Robot {
public:
  // TODO Include the fold the RobotService into the facade too.
  /**
   * Create a new `Robot` instance that will wrap the given `pose_service`
   * instance.
   *
   * The (squared) radius of the inner zone is set to a default value.
   */
  Robot(PoseService& pose_service)
      : pose_service_(pose_service),
        inner_zone_square_radius_(1.2) {}
  /**
   * Create a new `Robot` instance that will wrap the given `pose_service`
   * instance.
   *
   * The radius of the inner zone is set to `inner_zone_radius_`.
   */
  Robot(PoseService& pose_service, double inner_zone_radius)
      : pose_service_(pose_service),
        inner_zone_square_radius_(inner_zone_radius*inner_zone_radius) {}

  /**
   * Checks whether the given model is currently within the boundary of
   * the robot (i.e. the "inner zone").
   */
  bool isInRobotBoundary(lepp::ObjectModel const& model) const;

  /**
   * Returns the robot's position in the world coordinate system (i.e. ODO).
   */
  lepp::Coordinate robot_position() const { return pose_service_.getRobotPosition(); }

  double inner_zone_square_radius() const { return inner_zone_square_radius_; }
private:
  PoseService& pose_service_;
  /**
   * The (square of the) radius of the area around the robot that is considered
   * its "inner" zone.
   */
  double const inner_zone_square_radius_;
};

#endif
