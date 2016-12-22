#ifndef LEPP3_FILTER_SENSOR_TRUNCATE_FILTER_H__
#define LEPP3_FILTER_SENSOR_TRUNCATE_FILTER_H__

#include "lepp3/filter/PointFilter.hpp"

namespace {
  int genFactor(int decimals) {
    int factor = 1;
    for (int i = 0; i < decimals; ++i) factor *= 10;

    return factor;
  }
}

/**
 * Truncates the x, y, z coordinates of the given point to the given number of
 * decimal points.
 */
template<class PointT>
class TruncateFilter : public PointFilter<PointT> {
public:
  /**
   * Creates a new `TruncateFilter` that will truncate all coordinates to the
   * given number of decimal places.
   */
  TruncateFilter(int decimals) : factor_(genFactor(decimals)) {}
  /**
   * Implementation of the `PointFilter` interface.
   */
  bool apply(PointT& pt) {
    pt.x = static_cast<int>(pt.x * factor_) / static_cast<double>(factor_);
    pt.y = static_cast<int>(pt.y * factor_) / static_cast<double>(factor_);
    pt.z = static_cast<int>(pt.z * factor_) / static_cast<double>(factor_);
    return true;
  }

  void prepareNext() {}
  virtual const char* name() const override { return "TruncateFilter"; }

private:
  int const factor_;
};

#endif
