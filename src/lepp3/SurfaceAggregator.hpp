#ifndef LEPP2_SURFACE_AGGREGATOR_H__
#define LEPP2_SURFACE_AGGREGATOR_H__

#include <vector>

namespace lepp
{

/**
 * An interface that all classes that wish to be notified of surfaces detected
 * by an SurfaceDetector need to implement.
 *
 * The member function ``updateSurfaces`` accepts a new list of Surfaces
 * found by the detector.
 */
template<class PointT>
class SurfaceAggregator {
public:
  /**
   * The member function that all concrete aggregators need to implement in
   * order to be able to process newly detected surfaces.
   */
    virtual void updateSurfaces(std::vector<typename pcl::PointCloud<PointT>::ConstPtr> surfaces) = 0;
};

} // namespace lepp
#endif
