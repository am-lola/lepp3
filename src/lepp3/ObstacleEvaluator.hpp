#ifndef LEPP3_OBSTACLE_EVALUATOR_H_
#define LEPP3_OBSTACLE_EVALUATOR_H_

#include <boost/filesystem.hpp>
#include <iostream>
#include <sstream>

#include "lepp3/FrameData.hpp"
#include "lepp3/util/util.h"

/**
 * A class that computes the volume of a given model.
 * It is a `ModelVisitor` implementation and it assumes one model as input,
 * regardless of how many sub-parts the approximation has, a.k.a the
 * `CompositeModel`.
 *
 * The volume computation is done by creating a 3D grid around the model and
 * estimating how many points on the grid are occupied.
 */
class VolumeEstimator : public ModelVisitor {
public:
  VolumeEstimator()
      : num_splits_(0) {

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
  int getSplitCount() { return num_splits_; }
  /**
   * Estimates the volume of the approximated model. This is achieved by
   * creating a bounding box around the min/max points of the approximation and
   * subdivide the region into small grids.
   */
  int estimateVolume();

private:
  /**
   * Checks if the current point on the 3D grid (a.k.a the bounding box around
   * the model) is inside the model.
   */
  bool isVoxelOccupied(double const& x, double const& y, double const& z);
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
  /**
   * Number of sub-models in this model. This determines how many split operations
   * have been executed
   */
  int num_splits_;
};

void VolumeEstimator::visitSphere(lepp::SphereModel& sphere) {
  ++num_splits_;
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

void VolumeEstimator::visitCapsule(lepp::CapsuleModel& capsule) {
  ++num_splits_;
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

bool VolumeEstimator::isVoxelOccupied(
        double const& x, double const& y, double const& z) {

  // Go through all spheres of the current model and check if the point is
  // inside one of them.
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
  // Go through all capsules of the current model and check if the point is
  // inside one of them.
  {
    size_t sz = capsules_.size();
    for (size_t i=0; i<sz; ++i) {
      double r = capsules_[i].radius();
      /*
       * Prepare the pcl::sqrPointToLineDistance input arguments.
       */
      // line_dir: line direction vector
      Coordinate l = capsules_[i].second() - capsules_[i].first();
      Eigen::Vector4f line_dir(l.x, l.y, l.z, 0.);
      // line_pt: a point on the line
      Eigen::Vector4f line_pt(
              capsules_[i].second().x,
              capsules_[i].second().y,
              capsules_[i].second().z,
              0.);
      Eigen::Vector4f pt(x, y, z, 0);

      double const distance = pcl::sqrPointToLineDistance(pt, line_pt, line_dir);
      if (distance < r*r)
        return true;
    }
  }

  return false;
}

int VolumeEstimator::estimateVolume() {
  // NOTE: All the values are in METERS
  // Create a 3D grid and check the distance of each point on grid to the object
  // models.
  // TODO: 3D grid creation should depend on the point cloud resolution
  double const step_size = 0.01;
  int volume = 0;
  for (double x = min_p_.x; x < max_p_.x; x += step_size) {
    for (double y = min_p_.y; y < max_p_.y; y += step_size) {
      for (double z = min_p_.z; z < max_p_.z; z += step_size) {
        if (isVoxelOccupied(x, y, z))
          ++volume;
      }
    }
  }
  return volume;
}

/**
 *
 */
class ObstacleEvaluator : public FrameDataObserver {
public:
  ObstacleEvaluator();
  ObstacleEvaluator(int vol);

  /**
   * ObstacleAggregator interface implementation: processes the current models.
   */
  void updateFrame(FrameDataPtr frameData);
private:
  void init();
  bool evaluate(ObjectModelPtr const& model);
  std::string file_path_;
  int ref_volume_;
};

ObstacleEvaluator::ObstacleEvaluator()
    : ref_volume_(0) {

  init();
}

ObstacleEvaluator::ObstacleEvaluator(int vol)
    : ref_volume_(vol) {

  init();
}

void ObstacleEvaluator::init() {
  namespace bfs = boost::filesystem;
  std::stringstream ss;
  // Create the evaluation directory
  ss << "../evaluation/";
  std::string dir = ss.str();
  if ( !bfs::exists(bfs::path(dir)) )
    bfs::create_directory(bfs::path(dir));
  // Create the sub directory based on current timestamp
  ss << lepp::get_current_timestamp();
  dir = ss.str();
  bfs::create_directory(bfs::path(dir));
  // Prepare the csv file path
  ss  << "/eval.csv";
  file_path_ = ss.str();
  std::cout << "file_path: " << file_path_ << std::endl;
  // Create the file header
  std::ofstream tf_fout;
  tf_fout.open(file_path_.c_str(), std::ofstream::app);
  tf_fout << "model id,"
          << "splits,"
          << "est. volume,"
          << "reference volume,"
          << "ratio" << std::endl;
  tf_fout.close();
}

bool ObstacleEvaluator::evaluate(ObjectModelPtr const& model) {
  // TODO incorporate try-catch scheme
  VolumeEstimator estimator;
  // Find the minimum and maximum point of the approximated model
  model->accept(estimator);
  // Compute the volume of the current model
  int vol = estimator.estimateVolume();
  // Evaluate the approximation based on the reference information
  float ratio = -1;
  if (ref_volume_ != 0)
    ratio = vol / static_cast<float>(ref_volume_);
  // Save the current approximation information
  std::stringstream ss;
  ss << model->id() << ","
    << estimator.getSplitCount() << ","
    << vol << ","
    << ref_volume_ << ","
    << ratio << std::endl;
  std::ofstream tf_fout;
  // open the file and add the current model evaluation to the end of it
  tf_fout.open(file_path_.c_str(), std::ofstream::app);
  tf_fout << ss.str();
  tf_fout.close();

  return true;
}

void ObstacleEvaluator::updateFrame(FrameDataPtr frameData) {
  size_t sz = frameData->obstacles.size();
  for (size_t i=0; i<sz; ++i) {
    evaluate(frameData->obstacles[i]);
  }
}

#endif // LEPP3_OBSTACLE_EVALUATOR_H_
