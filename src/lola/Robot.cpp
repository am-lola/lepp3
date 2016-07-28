#include "lola/Robot.h"
#include "lepp3/models/Coordinate.h"
#include "lepp3/models/SurfaceModel.h"


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

// checks to see if the robot is near or on a given surface
// If the robot's center is within the surface's convex hull or near
// to one of the edges which forms the convex hull, this returns TRUE.
bool Robot::isInRobotBoundary(SurfaceModel const& model) const {
  int obj_id = model.id();

  Coordinate robot_position(0, 0, 0); // this->robot_position(); /// TODO: Re-enable this when ready to test with real robot!
  PointT robot_center;
  robot_center.x=robot_position.x;
  robot_center.y=robot_position.y;
  robot_center.z=robot_position.z;

  // check if robot is inside (or standing on) the surface
  bool isInside = pcl::isPointIn2DPolygon(robot_center, *(model.get_hull()));

  // check if robot is close to the edge of the surface
  double min_dist_to_poly = 10000000;
  for(int i = 1; i <= model.get_hull()->points.size(); i++)
  {
    double dist_to_poly = 10000000; // distance to this edge
    PointT point1 = model.get_hull()->points[i % model.get_hull()->points.size()];
    PointT point2 = model.get_hull()->points[i-1];

    Eigen::Vector3d p1_to_p2 = {(point1 - point2).x, (point1 - point2).y, (point1 - point2).z};
    Eigen::Vector3d robot_to_p1 = {(robot_center - point1).x, (robot_center - point1).y, (robot_center - point1).z}; // I hate myself

    auto r = p1_to_p2.dot(robot_to_p1);
    r /= robot_to_p1.norm();

    if (r < 0)
    {
      dist_to_poly = robot_to_p1.norm();
    }
    else if (r > 1)
    {
      dist_to_poly = sqrt(robot_to_p1.squaredNorm()
                         - r * p1_to_p2.squaredNorm());
    }

    min_dist_to_poly = std::min(dist_to_poly, min_dist_to_poly);
  }

  // The surface is considered to be in the robot's boundary if closer
  // than a particular threshold or the robot's position is within the
  // bounds of the surface's convex hull.
  return isInside || (min_dist_to_poly < inner_zone_square_radius_);
}
