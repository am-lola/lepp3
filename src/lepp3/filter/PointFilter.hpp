#ifndef LEPP3_FILTER_POINT_FILTER_H__
#define LEPP3_FILTER_POINT_FILTER_H__

namespace lepp {

template<class PointT>
class PointFilter {
public:
  virtual bool apply(PointT& pt) = 0;
  virtual void prepareNext() = 0;
};

}  // namespace lepp
#endif
