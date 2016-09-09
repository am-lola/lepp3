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
  CropFilter(double x_max, double x_min, double y_max, double y_min) : xmax(x_max), xmin(x_min), ymax(y_max), ymin(y_min) {}
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
  double const xmax;
  double const xmin;
  double const ymax;
  double const ymin;
};

#endif
