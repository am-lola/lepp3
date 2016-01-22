#ifndef lepp3_SURFACE_AGGREGATOR_H__
#define lepp3_SURFACE_AGGREGATOR_H__

#include "lepp3/Typedefs.hpp"
#include <pcl/ModelCoefficients.h>
#include <vector>
#include "lepp3/models/SurfaceModel.h"

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
    virtual void updateSurfaces(std::vector<SurfaceModelPtr> const& surfaces,
								PointCloudPtr &cloudMinusSurfaces, 
    							std::vector<pcl::ModelCoefficients> *&surfaceCoefficients) = 0;
};

} // namespace lepp
#endif
