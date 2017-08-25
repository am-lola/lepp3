#ifndef LEPP3_OBJECT_APPROXIMATOR_H__
#define LEPP3_OBJECT_APPROXIMATOR_H__

#include "lepp3/FrameData.hpp"

namespace lepp {

/**
 * An abstract base class that classes that wish to generate approximations
 * for point clouds need to implement.
 */
class ObjectApproximator : public FrameDataObserver, public FrameDataSubject {
public:
  ObjectApproximator(double max_squared_distance = 0.0075);

  virtual void updateFrame(FrameDataPtr frameData) override;

  /**
   * Generate the approximations for the object parameters (point cloud, etc).
   * The method assumes that the given point cloud segment is a single physical
   * object and tries to find the best approximations for this object, using its
   * own specific approximation method, and any hints given in the object_params.
   */
  virtual ObjectModelPtr approximate(ObjectModelParams const& object_params) = 0;

private:
  /**
   * Return false if obstacle is below any surface and its center is closer
   * than MAX_SQUARED_DISTANCE to the boundary of the considered surface hull.
   */
  bool isValidObstacle(ObjectModelPtr const& obstacle, std::vector<SurfaceModelPtr> const& frameData) const;

  double max_squared_distance_;
};

}

#endif
