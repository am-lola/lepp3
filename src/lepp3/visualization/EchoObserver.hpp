#ifndef LEPP3_VISUALIZATION_ECHO_OBSERVER_H__
#define LEPP3_VISUALIZATION_ECHO_OBSERVER_H__

#include "lepp3/Typedefs.hpp"
#include "lepp3/FrameData.hpp"

namespace lepp {

/**
 * Simple FrameDataObserver implementation that echoes the point cloud to a PCL
 * CloudViewer for visualization purposes.
 */
template<class PointT>
class EchoObserver : FrameDataObserver
{
public:
  EchoObserver() : viewer_("PCL Viewer") {}

  virtual void updateFrame(FrameDataPtr frameData) {
    viewer_.showCloud(frameData->cloud);
  }

private:
  pcl::visualization::CloudViewer viewer_;
};

}

#endif
