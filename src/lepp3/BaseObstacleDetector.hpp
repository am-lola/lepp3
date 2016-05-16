#ifndef BASE_OBSTACLE_DETECTOR_H_
#define BASE_OBSTACLE_DETECTOR_H_

#include <vector>
#include <algorithm>
#include <iostream>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/distances.h>

#include "lepp3/VideoObserver.hpp"
#include "lepp3/BaseSegmenter.hpp"
#include "lepp3/NoopSegmenter.hpp"
#include "lepp3/EuclideanPlaneSegmenter.hpp"
#include "lepp3/ObstacleAggregator.hpp"
#include "lepp3/ObjectApproximator.hpp"
#include "lepp3/MomentOfInertiaApproximator.hpp"
#include "lepp3/SplitApproximator.hpp"

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
 * A class that computes the volume of a given model.
 * It is a `ModelVisitor` implementation and it assumes one model as input,
 * regardless of how many sub-parts the approximation has, a.k.a the
 * `CompositeModel`.
 *
 * The volume computation is done by creating a 3D grid around the model and
 * estimating how many points on the grid are occupied.
 */
class ModelEvaluator : public ModelVisitor {
public:
  ModelEvaluator()
      : volume_(0) {

    min_p_.x = std::numeric_limits<int>::max();
    min_p_.y = std::numeric_limits<int>::max();
    min_p_.z = std::numeric_limits<int>::max();
    max_p_.x = std::numeric_limits<int>::min();
    max_p_.y = std::numeric_limits<int>::min();
    max_p_.z = std::numeric_limits<int>::min();
  }
  /**
   * Implementation of the `ModelVisitor` interface. It will draw the given
   * sphere onto the `PCLVisualizer` to which it holds a reference.
   */
  void visitSphere(lepp::SphereModel& sphere);
  void visitCapsule(lepp::CapsuleModel& capsule);
  /**
   * Estimates the volume of the approximated model. This is achieved by
   * creating a bounding box around the min/max points of the approximation and
   * subdivide the region into small grids.
   */
  double estimateVolume();
private:
  /**
   * Checks if the current point on the 3D grid (a.k.a the bounding box around
   * the model) is inside the model.
   */
  bool isVoxelOccupied(double const& x, double const& y, double const& z);
  /**
   * Estimated value of a model (in cubic centimeter)
   */
  int volume_;
  /**
   * Minimum/Maximum points of the bounding box surrounding the model.
   */
  Coordinate min_p_, max_p_;
  /**
   * Container to hold track of sphere models in a `CompositeModel`
   */
  std::vector<SphereModel> spheres_;
  /**
   * Container to hold track of capsule models in a `CompositeModel`
   */
  std::vector<CapsuleModel> capsules_;
};

void ModelEvaluator::visitSphere(lepp::SphereModel& sphere) {
  // Store the current Sphere model.
  spheres_.push_back(sphere);

  Coordinate const center = sphere.center_point();

  double const max_x = center.x + sphere.radius();
  double const max_y = center.y + sphere.radius();
  double const max_z = center.z + sphere.radius();

  double const min_x = center.x - sphere.radius();
  double const min_y = center.y - sphere.radius();
  double const min_z = center.z - sphere.radius();

  max_p_.x = std::max(max_p_.x, max_x);
  max_p_.y = std::max(max_p_.y, max_y);
  max_p_.z = std::max(max_p_.z, max_z);

  min_p_.x = std::min(min_p_.x, min_x);
  min_p_.y = std::min(min_p_.y, min_y);
  min_p_.z = std::min(min_p_.z, min_z);
}

void ModelEvaluator::visitCapsule(lepp::CapsuleModel& capsule) {
  // Store the current Capsule model.
  capsules_.push_back(capsule);

  Coordinate first = capsule.first();
  Coordinate second = capsule.second();

  Coordinate min, max;
  // Find minimum values between capsule.first, second and min_p_
  min.x = std::min(first.x, second.x);
  min.y = std::min(first.y, second.y);
  min.z = std::min(first.z, second.z);
  max.x = std::max(first.x, second.x);
  max.y = std::max(first.y, second.y);
  max.z = std::max(first.z, second.z);
  // Add the capsule radius to min/max
  min.x -= capsule.radius();
  min.y -= capsule.radius();
  min.z -= capsule.radius();
  max.x += capsule.radius();
  max.y += capsule.radius();
  max.z += capsule.radius();

  // Find the global min/max
  min_p_.x = std::min(min_p_.x, min.x);
  min_p_.y = std::min(min_p_.y, min.y);
  min_p_.z = std::min(min_p_.z, min.z);
  max_p_.x = std::max(max_p_.x, max.x);
  max_p_.y = std::max(max_p_.y, max.y);
  max_p_.z = std::max(max_p_.z, max.z);
}

bool ModelEvaluator::isVoxelOccupied(
        double const& x, double const& y, double const& z) {

  {
    size_t sz = spheres_.size();
    for (size_t i=0; i<sz; ++i) {
      double const r = spheres_[i].radius();
      Coordinate const& center = spheres_[i].center_point();
      double const distance =
              (center.x - x) * (center.x - x) + (center.y - y) * (center.y - y) + (center.z - z) * (center.z - z);
      if (distance <= r * r)
        return true;
    }
  }

  {
    size_t sz = capsules_.size();
    for (size_t i=0; i<sz; ++i) {
/*
      printf("estimating point to capsule distance\n");
      printf("min: %f, %f, %f\tmax: %f, %f, %f\n",
             min_p_.x,
             min_p_.y,
             min_p_.z,
             max_p_.x,
             max_p_.y,
             max_p_.z);
*/
      double r = capsules_[i].radius();
      /*
       * Prepare the pcl::sqrPointToLineDistance input arguments.
       */
      // line_dir: line direction vector
      Coordinate l = capsules_[i].second() - capsules_[i].first();
      Eigen::Vector4f line_dir(l.x, l.y, l.z, 0.);
//      line_dir[0] = (float)(capsules_[i].second().x - capsules_[i].first().x);
//      line_dir[1] = (float)(capsules_[i].second().y - capsules_[i].first().y);
//      line_dir[2] = (float)(capsules_[i].second().z - capsules_[i].first().z);
//      line_dir[3] = 0; // 4th component of a line in 3D is zero.
      // line_pt: a point on the line
      Eigen::Vector4f line_pt(
              capsules_[i].second().x,
              capsules_[i].second().y,
              capsules_[i].second().z,
              0.);
      Eigen::Vector4f pt(x, y, z, 0);

      double const distance = pcl::sqrPointToLineDistance(pt, line_pt, line_dir);

      /*PINFO << "capsule.first:" << capsules_[i].first();
      PINFO << "capsule.second:" << capsules_[i].second();
      printf("line_dir: %f, %f, %f\n", line_dir[0], line_dir[1], line_dir[2]);
      printf("distance squared: %f\t", distance);
      printf("radius squared: %f\n", r * r);*/
      if (distance < r*r) {
        // printf("GOTCHA!");
        return true;
      }

    }
  }

  return false;
}

double ModelEvaluator::estimateVolume() {
  // NOTE: All the values are in METERS
  // Create a 3D grid and check the distance of each point on grid to the object
  // models.
  // TODO: 3D grid creation should depend on the point cloud resolution
  double const step_size = 0.01;
  int v_counter = 0;

  for (double x = min_p_.x; x < max_p_.x; x += step_size) {
    for (double y = min_p_.y; y < max_p_.y; y += step_size) {
      for (double z = min_p_.z; z < max_p_.z; z += step_size) {
        if (isVoxelOccupied(x, y, z)) {
          ++volume_;
          v_counter++;
        }
      }
    }
  }
  PINFO << "occupied points: " << v_counter;

  return volume_;
}

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
class BaseObstacleDetector : public lepp::VideoObserver<PointT>,
                             public IObstacleDetector {
public:
  /**
   * Creates a new `BaseObstacleDetector` that will use the given
   * `ObjectApproximator` instance for generating approximations for detected
   * obstacles.
   */
  BaseObstacleDetector(boost::shared_ptr<ObjectApproximator<PointT> > approx);
  virtual ~BaseObstacleDetector() {}

  /**
   * VideoObserver interface method implementation.
   */
  virtual void notifyNewFrame(
      int idx,
      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud);
  /**
   * VideoObserver interface method implementation.
   */
  virtual void notifyNewFrame(
      int idx,
      const typename boost::shared_ptr<openni_wrapper::Image>& image);
  /**
   * VideoObserver interface method implementation.
   */
  void notifyNewFrame(int idx, const cv::Mat& image) {}


protected:
  /// Some convenience typedefs
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

private:
  typename pcl::PointCloud<PointT>::ConstPtr cloud_;

  boost::shared_ptr<BaseSegmenter<PointT> > segmenter_;
  boost::shared_ptr<ObjectApproximator<PointT> > approximator_;

  /**
   * Performs a new update of the obstacle approximations.
   * Triggered when the detector is notified of a new frame (i.e. point cloud).
   */
  void update();
  /**
   * Tries to compute the volume of the given model. This is achieved using
   * an instance of the `ModelEvaluator` class.
   */
  void evaluateApproximation(ObjectModelPtr const& model);
};

template<class PointT>
BaseObstacleDetector<PointT>::BaseObstacleDetector(
    boost::shared_ptr<ObjectApproximator<PointT> > approx)
      : approximator_(approx),
        segmenter_(new EuclideanPlaneSegmenter<PointT>()) {
  // TODO Allow for dependency injection.
}


template<class PointT>
void BaseObstacleDetector<PointT>::notifyNewFrame(
    int id,
    const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud) {
  cloud_ = point_cloud;
  try {
    update();
  } catch (...) {
    LERROR << "ObstacleDetector: Obstacle detection failed ...";
  }
}

template<class PointT>
void BaseObstacleDetector<PointT>::notifyNewFrame(
    int id,
    const typename boost::shared_ptr<openni_wrapper::Image>& image) {
      return; // RGB image data is not used for obstacle detection
}

template<class PointT>
void BaseObstacleDetector<PointT>::update() {
  Timer t;
  t.start();
  std::vector<PointCloudConstPtr> segments(segmenter_->segment(cloud_));

  // Iteratively approximate the segments
  size_t segment_count = segments.size();
  std::vector<ObjectModelPtr> models;
  for (size_t i = 0; i < segment_count; ++i) {
    models.push_back(approximator_->approximate(segments[i]));
    // Evaluate the approximation
    evaluateApproximation(models.back());
  }
  t.stop();
  PINFO << "Obstacle detection took " << t.duration();

  notifyObstacles(models);
}

template<class PointT>
void BaseObstacleDetector<PointT>::evaluateApproximation(ObjectModelPtr const& model) {
  ModelEvaluator evaluator;
  // Find the minimum and maximum point of the approximated model
  model->accept(evaluator);
  Timer t;
  // Compute the volume of the approximated model(s)
  t.start();
  double const v = evaluator.estimateVolume();
  t.stop();
  PINFO << "SSV evaluation took " << t.duration();
  PINFO << "SSV volume: " << v << " cm^3";
  // printf("Press enter to grab the next frame...\n");
  // std::cin.get();
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
