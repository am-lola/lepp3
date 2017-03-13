#ifndef LEPP3_OBJECT_APPROXIMATOR_MOMENT_OF_INERTIA_APPROXIMATOR_H__
#define LEPP3_OBJECT_APPROXIMATOR_MOMENT_OF_INERTIA_APPROXIMATOR_H__

#include "lepp3/Typedefs.hpp"
#include "ObjectApproximator.hpp"

namespace lepp {

/**
 * An implementation of the ObjectApproximator abstract base class that performs
 * approximations based on a heuristic that chooses one of the object models
 * based on the principal axes of the object.
 */
class MomentOfInertiaObjectApproximator : public ObjectApproximator {
public:
  ObjectModelPtr approximate(
      const PointCloudConstPtr& point_cloud);

private:
  // Private helper member functions for fitting individual models.
  // Takes a pointer to a model and a descriptor and sets the parameters of the
  // model so that it describes the point cloud with the given features in the
  // best way.
  // The implementations of the functions are pretty much a copy-paste of the
  // legacy code (ergo not very clean).
  // TODO Clean up the implementations of the functions (e.g. lots of unused variables)
  // TODO Refactor them in terms of the `ModelVisitor` API (`FittingVisitor`).
  void performFitting(boost::shared_ptr<SphereModel> sphere,
                      const PointCloudConstPtr& point_cloud,
                      Eigen::Vector3f mass_center,
                      std::vector <Eigen::Vector3f> const& axes);
  void performFitting(boost::shared_ptr<CapsuleModel> capsule,
                      const PointCloudConstPtr& point_cloud,
                      Eigen::Vector3f mass_center,
                      std::vector <Eigen::Vector3f> const& axes);
  /**
   * Returns a point representing an estimation of the position of the center
   * of mass for the given point cloud.
   */
  Eigen::Vector3f estimateMassCenter(
      const PointCloudConstPtr& point_cloud);
};

} // namespace lepp

#endif
