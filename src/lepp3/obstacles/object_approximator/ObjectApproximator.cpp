#include "ObjectApproximator.hpp"

#include "lepp3/ConvexHullDetector.hpp"

lepp::ObjectApproximator::ObjectApproximator(double max_squared_distance)
    : max_squared_distance_(max_squared_distance) {
}

void lepp::ObjectApproximator::updateFrame(FrameDataPtr frameData) {
  frameData->obstacles.clear();

  // iterate in reverse, we need to remove clouds for invalid obstacles
  for (size_t i = frameData->obstacleParams.size() - 1; i < frameData->obstacleParams.size(); --i) {
    ObjectModelPtr obstacle = approximate(frameData->obstacleParams[i]);

    // if the obstacle params contained a valid id, pass it on to this obstacle
    if (frameData->obstacleParams[i].id >= 0)
    {
      obstacle->set_id(frameData->obstacleParams[i].id);
    }

    if (isValidObstacle(obstacle, frameData->surfaces)) {
      frameData->obstacles.emplace_back(obstacle);
    } else {
      frameData->obstacleParams.erase(std::begin(frameData->obstacleParams) + i);
    }
  }

  // obstacles is in the wrong order because we iterated in reverse
  std::reverse(std::begin(frameData->obstacles), std::end(frameData->obstacles));

  notifyObservers(frameData);
}

bool lepp::ObjectApproximator::isValidObstacle(ObjectModelPtr const& obstacle,
                                               std::vector<SurfaceModelPtr> const& surfaces) const {
  const Coordinate& center = obstacle->center_point();
  for (auto const& surface : surfaces) {
    const pcl::ModelCoefficients& coeff = surface->get_planeCoefficients();
    double projZ = (-coeff.values[0] * center.x - coeff.values[1] * center.y - coeff.values[3]) / coeff.values[2];
    // if the obstacle is above the plane
    if (projZ < center.z)
      continue;

    // if distance to hull is smaller than delta, return false
    // iterate over boundary of convex hull of current surface
    PointCloudConstPtr currentHull = surface->get_hull();
    for (int j = 0; j < currentHull->size(); j++) {
      PointT seg1 = currentHull->at(j);
      PointT seg2 = currentHull->at((j + 1) % currentHull->size());

      // get shortest distance of point (center.x, center.y, projZ) to line segment defined by points seg1 <-> seg2
      PointT projCenter(center.x, center.y, projZ);
      PointT shortestVec;
      ConvexHullDetector::projectPointOntoLineSegment(seg1, seg2, projCenter, shortestVec);
      double shortestDist = ConvexHullDetector::getVecLengthSquared(shortestVec);

      // if obstacle is below the surface and closer to its hull boundary than MAX_SQUARED_DISTANCE, the obstacle is invalid
      if (shortestDist < max_squared_distance_)
        return false;
    }
  }
  return true;
}
