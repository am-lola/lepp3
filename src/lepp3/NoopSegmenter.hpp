#ifndef LEPP3_NOOP_SEGMENTER_H__
#define LEPP3_NOOP_SEGMENTER_H__

#include "lepp3/BaseSegmenter.hpp"

namespace lepp {

/**
 * An implementation of the segmenter interface, which simply returns the
 * original point cloud as the only segment.
 */
template<class PointT>
class NoopSegmenter : public BaseSegmenter<PointT> {
public:
  typedef typename pcl::PointCloud<PointT>::ConstPtr CloudConstPtr;

  virtual std::vector<typename pcl::PointCloud<PointT>::ConstPtr> segment(
      const typename pcl::PointCloud<PointT>::ConstPtr& cloud);
};

template<class PointT>
std::vector<typename pcl::PointCloud<PointT>::ConstPtr>
NoopSegmenter<PointT>::segment(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud) {
  std::vector<CloudConstPtr> ret;

  // The result contains only the original cloud. No segmentation is performed.
  ret.push_back(cloud);

  return ret;
};

} // namespace lepp

#endif
