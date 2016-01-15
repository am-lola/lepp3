#ifndef LEPP3_VISUALIZATION_ECHO_OBSERVER_H__
#define LEPP3_VISUALIZATION_ECHO_OBSERVER_H__

#include "lepp3/Typedefs.hpp"
#include "lepp3/VideoObserver.hpp"

namespace lepp {

/**
 * Simple VideoObserver implementation that echoes the point cloud to a PCL
 * CloudViewer for visualization purposes.
 */
template<class PointT>
class EchoObserver : public VideoObserver<PointT> {
public:
  EchoObserver() : viewer_("PCL Viewer") {}

  virtual void notifyNewFrame(
      int idx,
      const PointCloudConstPtr& pointCloud) {
    viewer_.showCloud(pointCloud);
  }

private:
  pcl::visualization::CloudViewer viewer_;
};

}

#endif
