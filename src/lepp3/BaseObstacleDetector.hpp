#ifndef BASE_OBSTACLE_DETECTOR_H_
#define BASE_OBSTACLE_DETECTOR_H_

#include <vector>
#include <algorithm>

#include <pcl/visualization/cloud_viewer.h>

#include "lepp3/BaseSegmenter.hpp"
#include "lepp3/NoopSegmenter.hpp"
#include "lepp3/EuclideanPlaneSegmenter.hpp"
#include "lepp3/ObstacleAggregator.hpp"
#include "lepp3/ObjectApproximator.hpp"
#include "lepp3/MomentOfInertiaApproximator.hpp"
#include "lepp3/SplitApproximator.hpp"
#include "lepp3/SurfaceAggregator.hpp"

#include "deps/easylogging++.h"

using namespace lepp;

#include "lepp3/debug/timer.hpp"

/**
 * A base class for obstacle detectors.
 *
 * Provides the ability for `ObstacleAggregator`s to attach to it and a
 * convenience protected method that sends a notification to all of them with a
 * given list of models.
 */
class IObstacleDetector {
public:
  /**
   * Attaches a new ObstacleAggregator, which will be notified of newly detected
   * obstacles by this detector.
   */
  void attachObstacleAggregator(
      boost::shared_ptr<ObstacleAggregator> aggregator);

protected:
  /**
   * Notifies any observers about newly detected obstacles.
   */
  void notifyObstacles(std::vector<ObjectModelPtr> const& models);

private:
  /**
   * Tracks all attached ObstacleAggregators that wish to be notified of newly
   * detected obstacles.
   */
  std::vector<boost::shared_ptr<ObstacleAggregator> > aggregators_;
};

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
class BaseObstacleDetector : public IObstacleDetector,
                             public SurfaceAggregator<PointT>{
public:
  /**
   * Creates a new `BaseObstacleDetector` that will use the given
   * `ObjectApproximator` instance for generating approximations for detected
   * obstacles.
   */
  BaseObstacleDetector(boost::shared_ptr<ObjectApproximator<PointT> > approx);
  virtual ~BaseObstacleDetector() {}


  virtual void updateSurfaces(std::vector<typename pcl::PointCloud<PointT>::ConstPtr> surfaces,
      typename pcl::PointCloud<PointT>::Ptr &cloudMinusSurfaces);

protected:
  /// Some convenience typedefs
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

private:
  typename pcl::PointCloud<PointT>::Ptr cloud_;

  boost::shared_ptr<BaseSegmenter<PointT> > segmenter_;
  boost::shared_ptr<ObjectApproximator<PointT> > approximator_;

  /**
   * Performs a new update of the obstacle approximations.
   * Triggered when the detector is notified of a new frame (i.e. point cloud).
   */
  void update();
};

template<class PointT>
BaseObstacleDetector<PointT>::BaseObstacleDetector(
    boost::shared_ptr<ObjectApproximator<PointT> > approx)
      : approximator_(approx),
        segmenter_(new EuclideanPlaneSegmenter<PointT>()) {
  // TODO Allow for dependency injection.
}


template<class PointT>
void BaseObstacleDetector<PointT>::updateSurfaces(std::vector<typename pcl::PointCloud<PointT>::ConstPtr> surfaces,
      typename pcl::PointCloud<PointT>::Ptr &cloudMinusSurfaces) {
  cloud_ = cloudMinusSurfaces;
  try {
    update();
  } catch (...) {
    LERROR << "ObstacleDetector: Obstacle detection failed ...";
  }
}

template<class PointT>
void BaseObstacleDetector<PointT>::update() {
  Timer t;
  t.start();
  typename pcl::PointCloud<PointT>::ConstPtr dummyCloud(new pcl::PointCloud<PointT>());
  std::vector<typename pcl::PointCloud<PointT>::ConstPtr> segments;
  segmenter_->segment(dummyCloud, segments, cloud_);

  // Iteratively approximate the segments
  size_t segment_count = segments.size();
  std::vector<ObjectModelPtr> models;
  for (size_t i = 0; i < segment_count; ++i) {
    models.push_back(approximator_->approximate(segments[i]));
  }
  t.stop();
  //PINFO << "Obstacle detection took " << t.duration();

  notifyObstacles(models);
}

void IObstacleDetector::attachObstacleAggregator(
    boost::shared_ptr<ObstacleAggregator> aggregator) {
  aggregators_.push_back(aggregator);
}

void IObstacleDetector::notifyObstacles(
    std::vector<ObjectModelPtr> const& models) {
  size_t sz = aggregators_.size();
  for (size_t i = 0; i < sz; ++i) {
    aggregators_[i]->updateObstacles(models);
  }
}

#endif
