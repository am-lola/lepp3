#include "lola/Robot.h"
#include "lepp3/models/Coordinate.h"

using namespace lepp;

bool Robot::isInRobotBoundary(ObjectModel const& model) const {
  int obj_id = model.id();
  Coordinate model_center = model.center_point();
  Coordinate robot_position(0, 0, 0); // this->robot_position(); /// TODO: Re-enable this when ready to test with real robot!
  // Now find the distance between the two coordinates, giving the
  // (rough) distance between the robot and the object.
  double const squared_dist = (model_center - robot_position).square_norm();

  // The object is considered to be in the robot's boundary if closer
  // than a particular threshold.
  return squared_dist < inner_zone_square_radius_;
}
