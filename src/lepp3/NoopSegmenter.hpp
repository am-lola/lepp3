#ifndef LEPP3_NOOP_SEGMENTER_H__
#define LEPP3_NOOP_SEGMENTER_H__

#include "lepp3/Typedefs.hpp"
#include "lepp3/BaseSegmenter.hpp"

namespace lepp {

/**
 * An implementation of the segmenter interface, which simply returns the
 * original point cloud as the only segment.
 */
template<class PointT>
class NoopSegmenter : public BaseSegmenter<PointT> {
public:
  virtual std::vector<PointCloundConstPtr> segment(
      const PointCloundConstPtr& cloud);
};

template<class PointT>
std::vector<PointCloundConstPtr>
NoopSegmenter<PointT>::segment(
    const PointCloundConstPtr& cloud) {
  std::vector<PointCloundConstPtr> ret;

  // The result contains only the original cloud. No segmentation is performed.
  ret.push_back(cloud);

  return ret;
};

} // namespace lepp

#endif
