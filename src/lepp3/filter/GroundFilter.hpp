#ifndef LEPP3_FILTER_SENSOR_GROUND_FILTER_H__
#define LEPP3_FILTER_SENSOR_GROUND_FILTER_H__

#include "lepp3/filter/PointFilter.hpp"


/**
 * Removes points that are around z=0, with a defined threshold
 */
template<class PointT>
class GroundFilter : public PointFilter<PointT> {
public:
  /**
   *
   */
  GroundFilter(double threshold) : threshold_(threshold) {}
  /**
   * Implementation of the `PointFilter` interface.
   */
  bool apply(PointT& pt) {
    if (std::abs(pt.z) < threshold_)
      return false;
    else
      return true;
  }

  void prepareNext() {}

  virtual int order() const override { return 2; }
  virtual const char* name() const override { return "GroundFilter"; }
  virtual std::vector<std::string> dependencies() const override { return { "RobotOdoTransformer" }; }
private:
  double const threshold_;
};

#endif
