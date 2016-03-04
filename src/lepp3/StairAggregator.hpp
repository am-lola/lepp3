#ifndef LEPP3_STAIR_AGGREGATOR_H__
#define LEPP3_STAIR_AGGREGATOR_H__

#include <vector>

namespace lepp
{

/**
 * An interface that all classes that wish to be notified of stairs detected
 * by an StairDetector need to implement.
 *
 * The member function ``updateStairs`` accepts a new list of Stairs
 * found by the detector.
 */
template<class PointT>
class StairAggregator {
public:
  /**
   * The member function that all concrete aggregators need to implement in
   * order to be able to process newly detected obstacles.
   */
    virtual void updateStairs(std::vector<typename pcl::PointCloud<PointT>::ConstPtr> stairs) = 0;
};

} // namespace lepp
#endif
