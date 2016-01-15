
#ifndef LEPP3_CONVEX_HULL_AGGREGATOR_H__
#define LEPP3_CONVEX_HULL_AGGREGATOR_H__

#include "lepp3/Typedefs.hpp"
#include <pcl/ModelCoefficients.h>
#include <vector>

namespace lepp
{
template<class PointT>
class ConvexHullAggregator {
public:
    virtual void updateHulls(std::vector<PointCloudConstPtr> &hulls) = 0;
};

} // namespace lepp
#endif
