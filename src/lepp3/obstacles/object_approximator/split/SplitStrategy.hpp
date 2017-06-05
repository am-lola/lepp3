#ifndef LEPP3_OBJECT_APPROXIMATOR_SPLIT_SPLIT_STRATEGY_H__
#define LEPP3_OBJECT_APPROXIMATOR_SPLIT_SPLIT_STRATEGY_H__

#include <vector>

#include "lepp3/Typedefs.hpp"

namespace lepp {

/**
 * An ABC that represents the strategy for splitting a point cloud used by the
 * `SplitObjectApproximator`.
 *
 * Varying the `SplitStrategy` implementation allows us to change how point
 * clouds are split (or if they are split at all) without changing the logic of
 * the `SplitObjectApproximator` approximator itself.
 */
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
  virtual std::vector <PointCloudPtr> split(
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
  virtual std::vector <PointCloudPtr> doSplit(
      const PointCloudConstPtr& point_cloud);

private:
  SplitAxis axis_;
};

}

#endif
