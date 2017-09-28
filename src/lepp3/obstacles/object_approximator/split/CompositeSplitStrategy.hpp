#ifndef LEPP3_OBJECT_APPROXIMATOR_SPLIT_COMPOSITE_SPLIT_STRATEGY_H__
#define LEPP3_OBJECT_APPROXIMATOR_SPLIT_COMPOSITE_SPLIT_STRATEGY_H__

#include "SplitStrategy.hpp"
#include "SplitCondition.hpp"

namespace lepp {

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
class CompositeSplitStrategy : public SplitStrategy {
public:
  void addSplitCondition(std::shared_ptr<SplitCondition> cond) {
    conditions_.push_back(cond);
  }

private:
  bool shouldSplit(
      int split_depth,
      const PointCloudConstPtr& point_cloud);

  /**
   * A list of conditions that will be checked before any split happens.
   */
  std::vector<std::shared_ptr<SplitCondition>> conditions_;
};
}

#endif
