#ifndef LEPP3_FILTER_CLOUD_PRE_DOWNSAMPLE_FILTER_H__
#define LEPP3_FILTER_CLOUD_PRE_DOWNSAMPLE_FILTER_H__

#include "lepp3/filter/cloud/pre/CloudPreFilter.hpp"
#include <pcl/filters/voxel_grid.h>

namespace lepp {

template<class PointT>
class DownsampleFilter : public CloudPreFilter<PointT> {
public:
  DownsampleFilter(double cube_size)
    : cube_size_(cube_size) {}

  virtual void newFrame() override {}
  virtual void getFiltered(PointCloudConstPtr& filtered) override {
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(filtered);
    sor.setLeafSize(cube_size_, cube_size_, cube_size_);

    PointCloudPtr cloud_filtered(new PointCloudT());
    sor.filter(*cloud_filtered);

    filtered = cloud_filtered;
  }

private:
  const double cube_size_;
};

}

#endif
