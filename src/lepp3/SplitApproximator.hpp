#ifndef LEPP3_SPLIT_APPROXIMATOR_H__
#define LEPP3_SPLIT_APPROXIMATOR_H__

#include "lepp3/Typedefs.hpp"
#include "lepp3/ObjectApproximator.hpp"
#include "lepp3/models/ObjectModel.h"

#include <deque>
#include <map>

#include <pcl/common/pca.h>
#include <pcl/common/common.h>

namespace lepp {

/**
 * An ABC that represents the strategy for splitting a point cloud used by the
 * `SplitObjectApproximator`.
 *
 * Varying the `SplitStrategy` implementation allows us to change how point
 * clouds are split (or if they are split at all) without changing the logic of
 * the `SplitObjectApproximator` approximator itself.
 */
template<class PointT>
class SplitStrategy {
public:
  enum SplitAxis {
    Largest = 0,
    Middle = 1,
    Smallest = 2,
  };

  SplitStrategy() : axis_(Largest) {}

  void set_split_axis(SplitAxis axis) { axis_ = axis; }
  SplitAxis split_axis() const { return axis_; }

  /**
   * Performs the split of the given point cloud according to the particular
   * strategy.
   *
   * This method has a default implementation that is a template method, which
   * calls the protected `shouldSplit` method and calls the `doSplit` if it
   * indicated that a split should be made. Since all of the methods are
   * virtual, concrete implementations can override how the split is made or
   * keep the default implementation.
   *
   * This method can also be overridden by concrete implementations if they
   * cannot implement the logic only in terms of the `shouldSplit` and
   * `doSplit` method implementations (although that should be rare).
   *
   * :param split_depth: The current split depth, i.e. the number of times the
   *     original cloud has already been split
   * :param point_cloud: The current point cloud that should be split by the
   *    `SplitStrategy` implementation.
   * :returns: The method should return a vector of point clouds obtained by
   *      splitting the given cloud into any number of parts. If the given
   *      point cloud should not be split, an empty vector should be returned.
   *      Once the empty vector is returned, the `SplitObjectApproximator` will
   *      stop the splitting process for that branch of the split tree.
   */
  virtual std::vector<PointCloudPtr> split(
      int split_depth,
      const PointCloudConstPtr& point_cloud);
protected:
  /**
   * A pure virtual method that decides whether the given point cloud should be
   * split or not.
   *
   * :param split_depth: The current split depth, i.e. the number of times the
   *     original cloud has already been split
   * :param point_cloud: The current point cloud that should be split by the
   *    `SplitStrategy` implementation.
   * :returns: A boolean indicating whether the cloud should be split or not.
   */
  virtual bool shouldSplit(
      int split_depth,
      const PointCloudConstPtr& point_cloud) = 0;
  /**
   * A helper method that does the actual split, when needed.
   * A default implementation is provided, since that is what most splitters
   * will want to use...
   */
  virtual std::vector<PointCloudPtr> doSplit(
      const PointCloudConstPtr& point_cloud);
private:
  SplitAxis axis_;
};

template<class PointT>
std::vector<PointCloudPtr> SplitStrategy<PointT>::split(
    int split_depth,
    const PointCloudConstPtr& point_cloud) {
  if (this->shouldSplit(split_depth, point_cloud)) {
    return this->doSplit(point_cloud);
  } else {
    return std::vector<PointCloudPtr>();
  }
}

template<class PointT>
std::vector<PointCloudPtr>
SplitStrategy<PointT>::doSplit(
    const PointCloudConstPtr& point_cloud) {
  // Compute PCA for the input cloud
  pcl::PCA<PointT> pca;
  pca.setInputCloud(point_cloud);
  Eigen::Vector3f eigenvalues = pca.getEigenValues();
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();

  Eigen::Vector3d main_pca_axis = eigenvectors.col(static_cast<int>(axis_))
                                              .cast<double>();

  // Compute the centroid
  Eigen::Vector4d centroid;
  pcl::compute3DCentroid(*point_cloud, centroid);

  /// The plane equation
  double d = (-1) * (
      centroid[0] * main_pca_axis[0] +
      centroid[1] * main_pca_axis[1] +
      centroid[2] * main_pca_axis[2]
  );

  // Prepare the two parts.
  std::vector<PointCloudPtr> ret;
  ret.push_back(PointCloudPtr(new PointCloudT()));
  ret.push_back(PointCloudPtr(new PointCloudT()));
  PointCloudT& first = *ret[0];
  PointCloudT& second = *ret[1];

  // Now divide the input cloud into two clusters based on the splitting plane
  size_t const sz = point_cloud->size();
  for (size_t i = 0; i < sz; ++i) {
    // Boost the precision of the points we are dealing with to make the
    // calculation more precise.
    PointT const& original_point = (*point_cloud)[i];
    Eigen::Vector3f const vector_point = original_point.getVector3fMap();
    Eigen::Vector3d const point = vector_point.cast<double>();
    // Decide on which side of the plane the current point is and add it to the
    // appropriate partition.
    if (point.dot(main_pca_axis) + d < 0.) {
      first.push_back(original_point);
    } else {
      second.push_back(original_point);
    }
  }

  // Return the parts in a vector, as expected by the interface...
  return ret;
}

/**
 * An ABC for classes that provide the functionality of checking whether a
 * point cloud should be split or not.
 */
template<class PointT>
class SplitCondition {
public:
  /**
   * A pure virtual method that decides whether the given point cloud should be
   * split or not.
   *
   * :param split_depth: The current split depth, i.e. the number of times the
   *     original cloud has already been split
   * :param point_cloud: The current point cloud that should be split by the
   *    `SplitStrategy` implementation.
   * :returns: A boolean indicating whether the cloud should be split or not.
   */
  virtual bool shouldSplit(
      int split_depth,
      const PointCloudConstPtr& point_cloud) = 0;
};

/**
 * An implementation of the `SplitStrategy` abstract base class that decides
 * whether a point cloud should be split by making sure that each condition
 * is satisfied (i.e. each `SplitCondition` instance returns `true` from its
 * `shouldSplit` method).
 *
 * If and only if all conditions are satisfied, the strategy performs the split
 * by using the default split implementation provided by the `SplitStrategy`
 * superclass.
 */
template<class PointT>
class CompositeSplitStrategy : public SplitStrategy<PointT> {
public:
  void addSplitCondition(boost::shared_ptr<SplitCondition<PointT> > cond) {
    conditions_.push_back(cond);
  }
protected:
  bool shouldSplit(
      int split_depth,
      const PointCloudConstPtr& point_cloud) {
    size_t const sz = conditions_.size();
    if (sz == 0) {
      // If there are no conditions, do not split the object, in order to avoid
      // perpetually splitting it, since there's no condition that could
      // possibly put an end to it.
      return false;
    }
    for (size_t i = 0; i < sz; ++i) {
      if (!conditions_[i]->shouldSplit(split_depth, point_cloud)) {
        // No split can happen if any of the conditions disallows it.
        return false;
      }
    }

    // Split only if all of the conditions allowed us to split
    return true;
  }
private:
  /**
   * A list of conditions that will be checked before any split happens.
   */
  std::vector<boost::shared_ptr<SplitCondition<PointT> > > conditions_;
};

/**
 * A `SplitCondition` that allows the split to be made only if the split depth
 * has not exceeded the given limit.
 */
template<class PointT>
class DepthLimitSplitCondition : public SplitCondition<PointT> {
public:
  DepthLimitSplitCondition(int depth_limit) : limit_(depth_limit) {}
  bool shouldSplit(
      int split_depth,
      const PointCloudConstPtr& point_cloud) {
    return split_depth < limit_;
  }
private:
  int const limit_;
};

/**
 * A `SplitCondition` that allows the split to be made only if the object's
 * volume is larger than the given limit.
 */
template<class PointT>
class SizeLimitSplitCondition : public SplitCondition<PointT> {
public:
  /**
   * Creates a new `SizeLimitSplitCondition`. The size limit that is given is
   * the minimum volume of the object (in **cubic centimeters**) for which a
   * split will be allowed.
   */
  SizeLimitSplitCondition(int size_limit) : limit_(size_limit) {}
  bool shouldSplit(
      int split_depth,
      const PointCloudConstPtr& point_cloud) {
    // Find the limits of the bounding box of the cloud
    PointT min_pt;
    PointT max_pt;
    pcl::getMinMax3D(*point_cloud, min_pt, max_pt);

    // Calculate the volume of the box bounded by those two points.
    // Make sure the units are centimeters.
    Coordinate const sz = 100 * (Coordinate(max_pt) - Coordinate(min_pt));
    int const volume = static_cast<int>(
        (sz.x * sz.x * sz.x) + (sz.y * sz.y * sz.y) + (sz.z * sz.z * sz.z));

    // Split only if the volume is larger than the limit
    return volume > limit_;
  }
private:
  int const limit_;
};

/**
 * A `SplitCondition` that stops the splitting of objects that are certainly a
 * very regular sphere or a very regular capsule. This is because there is no
 * point in splitting those further, instead of simply approximating them with
 * a single instance of that shape.
 */
template<class PointT>
class ShapeSplitCondition : public SplitCondition<PointT> {
public:
  ShapeSplitCondition()
      : sphere1(.7), sphere2(.1),
        capsule(.25) {}
  ShapeSplitCondition(double sphere1, double sphere2, double capsule)
      : sphere1(sphere1), sphere2(sphere2), capsule(capsule) {}
  bool shouldSplit(
      int split_depth,
      const PointCloudConstPtr& point_cloud) {
    float major_value, middle_value, minor_value;
    pcl::PCA<PointT> pca;
    pca.setInputCloud(point_cloud);
    Eigen::Vector3f eigenvalues = pca.getEigenValues();
    major_value = eigenvalues(0);
    middle_value = eigenvalues(1);
    minor_value = eigenvalues(2);

    if ((middle_value / major_value > sphere1) && (minor_value / major_value > sphere2)) {
      // This is very much a sphere, so we don't split it any more.
      return false;
    }

    if (middle_value / major_value < capsule) {
      // This is very much a capsule: stop the splitting.
      return false;
    }

    // The object didn't fall squarely into any of the known categories, so for
    // all it's concerned, the object could be split.
    return true;
  }
private:
  double sphere1;
  double sphere2;
  double capsule;
};
/**
 * An approximator implementation that will generate an approximation by
 * splitting the given object into multiple parts. Each part approximation is
 * generated by delegating to a wrapped `ObjectApproximator` instance, allowing
 * clients to vary the algorithm used for approximations, while keeping the
 * logic of incrementally splitting up the object.
 */
template<class PointT>
class SplitObjectApproximator : public ObjectApproximator<PointT> {
public:
  /**
   * Create a new `SplitObjectApproximator` that will approximate each part by
   * using the given approximator instance and perform splits decided by the
   * given `SplitStrategy` instance.
   */
  SplitObjectApproximator(
        boost::shared_ptr<ObjectApproximator<PointT> > approx,
        boost::shared_ptr<SplitStrategy<PointT> > splitter)
          : approximator_(approx),
            splitter_(splitter) {}
  /**
   * `ObjectApproximator` interface method.
   */
  boost::shared_ptr<CompositeModel> approximate(
      const PointCloudConstPtr& point_cloud);
private:
  /**
   * An `ObjectApproximator` used to generate approximations for object parts.
   */
  boost::shared_ptr<ObjectApproximator<PointT> > approximator_;
  /**
   * The strategy to be used for splitting point clouds.
   */
  boost::shared_ptr<SplitStrategy<PointT> > splitter_;
};

template<class PointT>
boost::shared_ptr<CompositeModel> SplitObjectApproximator<PointT>::approximate(
    const PointCloudConstPtr& point_cloud) {
  boost::shared_ptr<CompositeModel> approx(new CompositeModel);
  std::deque<std::pair<int, PointCloudConstPtr> > queue;
  queue.push_back(std::make_pair(0, point_cloud));

  while (!queue.empty()) {
    int const depth = queue[0].first;
    PointCloudConstPtr const current_cloud = queue[0].second;
    queue.pop_front();

    // Delegates to the wrapped approximator for each part's approximation.
    ObjectModelPtr model = approximator_->approximate(current_cloud);
    // TODO Decide whether the model fits well enough for the current cloud.
    // For now we fix the number of split iterations.
    // The approximation should be improved. Try doing it for the split clouds
    std::vector<PointCloudPtr> const splits = splitter_->split(depth, current_cloud);
    // Add each new split section into the queue as children of the current
    // node.
    if (splits.size() != 0) {
      for (size_t i = 0; i < splits.size(); ++i) {
        queue.push_back(std::make_pair(depth + 1, splits[i]));
      }
    } else {
      // Keep the approximation
      approx->addModel(model);
    }
  }

  return approx;
}

}  // namespace lepp
#endif
