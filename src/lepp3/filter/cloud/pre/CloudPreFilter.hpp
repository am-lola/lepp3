#ifndef LEPP3_FILTER_CLOUD_PRE_CLOUD_PRE_FILTER_H__
#define LEPP3_FILTER_CLOUD_PRE_CLOUD_PRE_FILTER_H__

namespace lepp {

template<class PointT>
class CloudPreFilter {
public:
  virtual void newFrame() = 0;
  virtual void getFiltered(PointCloudConstPtr& filtered) = 0;
};

}

#endif
