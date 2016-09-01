#ifndef LEPP3_FILTER_SENSOR_CROP_FILTER_H__
#define LEPP3_FILTER_SENSOR_CROP_FILTER_H__

#include "lepp3/filter/PointFilter.hpp"


/**
 * Crops points that are outside a defined lab area
 */
template<class PointT>
class CropFilter : public PointFilter<PointT> {
public:
  /**
   *
   */
  CropFilter(float x_max, float x_min, float y_max, float y_min) : xmax(x_max), xmin(x_min), ymax(y_max), ymin(y_min) {}
  /**
   * Implementation of the `PointFilter` interface.
   */
  bool apply(PointT& pt) {
    if (pt.x < xmax && pt.x > xmin && pt.y < ymax && pt.y > ymin)
      return true;
    else
      return false;
  }

  void prepareNext() {}
private:
  float const xmax;
  float const xmin;
  float const ymax;
  float const ymin;
};

#endif
