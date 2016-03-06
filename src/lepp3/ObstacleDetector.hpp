#ifndef BASE_OBSTACLE_DETECTOR_H_
#define BASE_OBSTACLE_DETECTOR_H_

#include "lepp3/Typedefs.hpp"
#include "lepp3/models/Coordinate.h"
#include "lepp3/BaseSegmenter.hpp"
#include "lepp3/EuclideanPlaneSegmenter.hpp"
#include "lepp3/ObjectApproximator.hpp"
#include "lepp3/FrameDataObserver.hpp"
#include "lepp3/FrameDataSubject.hpp"
#include "lepp3/models/ObjectModel.h"
#include "lepp3/ConvexHullDetector.hpp"

using namespace lepp;

/**
 * A basic implementation of an obstacle detector that detects obstacles from a
 * `VideoSource`. In order to do so, it needs to be attached to a `VideoSource`
 * instance (and therefore it implements the `VideoObserver` interface).
 *
 * Obstacles in each frame that the `VideoSource` gives to the detector are
 * found by first performing segmentation of the given point cloud (using the
 * given `BaseSegmenter` instance), followed by performing the approximation
 * of each of them by the given `ObjectApproximator` instance.
 */
template<class PointT>
class ObstacleDetector : public FrameDataObserver, public FrameDataSubject 
{
public:
  /**
   * Creates a new `ObstacleDetector` that will use the given
   * `ObjectApproximator` instance for generating approximations for detected
   * obstacles.
   */
  ObstacleDetector(boost::shared_ptr<ObjectApproximator<PointT> > approx);
  virtual ~ObstacleDetector() {}

  virtual void updateFrame(FrameDataPtr frameData);

private:
  PointCloudPtr cloud_;
  boost::shared_ptr<BaseSegmenter<PointT> > segmenter_;
  boost::shared_ptr<ObjectApproximator<PointT> > approximator_;
  static const double MAX_SQUARED_DISTANCE = 0.0075;


  /**
  * Return false if obstalce is below any surface and its center is closer 
  * than MAX_SQUARED_DISTANCE to the boundary of the considered surface hull.
  */
  bool isValidObstacle(FrameDataPtr frameData, int obstacleIndex);
};

template<class PointT>
ObstacleDetector<PointT>::ObstacleDetector(
    boost::shared_ptr<ObjectApproximator<PointT> > approx)
      : approximator_(approx),
        segmenter_(new EuclideanPlaneSegmenter<PointT>()) 
{}


template<class PointT>
bool ObstacleDetector<PointT>::isValidObstacle(FrameDataPtr frameData, int obstacleIndex)
{
  const Coordinate &center = frameData->obstacles[obstacleIndex]->center_point();
  for(int i = 0; i < frameData->surfaces.size(); i++)
  {
    const pcl::ModelCoefficients &coeff = frameData->surfaces[i]->get_planeCoefficients();
    double projZ = (-coeff.values[0] * center.x - coeff.values[1] * center.y - coeff.values[3]) / coeff.values[2];
    // if the obstacle is above the plane, PAY ATTENTION TO THE Z-COORDINATE DIRECTION!
    if (projZ > center.z)
      continue;

    // if distance to hull is smaller than delta, return false
    // iterate over boundary of convex hull of current surface
    PointCloudConstPtr currentHull = frameData->surfaces[i]->get_hull();
    for (int j = 0; j < currentHull->size(); j++)
    {
      PointT seg1 = currentHull->at(j);
      PointT seg2 = currentHull->at((j+1) % currentHull->size());
      
      // get shortest distance of point (center.x, center.y, projZ) to line segment defined by points seg1 <-> seg2
      PointT projCenter(center.x, center.y, projZ);
      PointT shortestVec;
      ConvexHullDetector::projectPointOntoLineSegment(seg1, seg2, projCenter, shortestVec);
      double shortestDist = ConvexHullDetector::getVecLengthSquared(shortestVec);
     
      // if obstacle is belove the surface and closer to its hull boundary than MAX_SQUARED_DISTANCE, the obstacle is invalid
      if (shortestDist < MAX_SQUARED_DISTANCE)
        return false;
    }
  }
  return true;
}


template<class PointT>
void ObstacleDetector<PointT>::updateFrame(FrameDataPtr frameData) 
{
  segmenter_->segment(frameData);

  // Iteratively approximate the segments
  for (size_t i = 0; i < frameData->obstacleClouds.size(); i++) 
  {
    frameData->obstacles.push_back(approximator_->approximate(frameData->obstacleClouds[i]));
  }
  
  // remove invalid obstacles
  std::vector<ObjectModelPtr> validObstacles;
  std::vector<PointCloudPtr> validObstacleClouds;
  for (size_t i = 0; i < frameData->obstacles.size(); i++)
  {
    if (isValidObstacle(frameData,i))
    {
      validObstacles.push_back(frameData->obstacles[i]);
      validObstacleClouds.push_back(frameData->obstacleClouds[i]);
    }
  }
  frameData->obstacles = validObstacles;
  frameData->obstacleClouds = validObstacleClouds;
  
  notifyObservers(frameData);
}

#endif
