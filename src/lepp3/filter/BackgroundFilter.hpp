#ifndef LEPP3_FILTER_SENSOR_BACKGROUND_FILTER_H__
#define LEPP3_FILTER_SENSOR_BACKGROUND_FILTER_H__

#include "lepp3/filter/PointFilter.hpp"


/**
 * Removes points that are far away from the camera, with a defined threshold
 */
template<class PointT>
class BackgroundFilter : public PointFilter<PointT> {
public:
  /**
   *
   */
  BackgroundFilter(double threshold) : threshold_(threshold) {}
  /**
   * Implementation of the `PointFilter` interface.
   */
  bool apply(PointT& pt) {
    if (pt.z < threshold_)
      return true;
    else
      return false;
  }

  void prepareNext() {}

  virtual int order() const override { return -4; }
  virtual const char* name() const override { return "BackgroundFilter"; }

private:
  double const threshold_;
};

#endif
