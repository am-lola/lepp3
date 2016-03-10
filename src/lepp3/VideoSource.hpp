#ifndef BASE_VIDEO_SOURCE_H_
#define BASE_VIDEO_SOURCE_H_

#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include "lepp3/FrameData.hpp"
#include "lepp3/RGBData.hpp"
#include "lepp3/Typedefs.hpp"

namespace lepp {

/**
 * The abstract base class for all classes that wish to be sources of point
 * cloud video information.
 *
 * The VideoSource is an "Observable" object to which interested observers
 * (implementing the FrameDataObserver interface) can attach.
 *
 * This class provides the common functionality for all VideoSource
 * implementations. Most importantly, it implements all logic regarding tracking
 * and notifying observers of the source when a new point cloud is received by
 * the source.
 */
template<class PointT>
class VideoSource : public FrameDataSubject, public RGBDataSubject {
public:
  VideoSource() {}
  virtual ~VideoSource();

  /**
   * Starts the video source.
   */
  virtual void open() = 0;

  /**
   * Convenience method for subclasses to set any option on the members or
   * methods of the class.
   */
  virtual void setOptions(
    std::map<std::string, bool> options) = 0;

protected:
  /**
   * Convenience method for subclasses to indicate that a new cloud has been
   * received.  It is enough to invoke this method in order to register a new
   * point cloud as the most recent one, as well as to notify all source
   * observers.
   */
  virtual void setNextFrame(FrameDataPtr frameData);

  virtual void setNextFrame(RGBDataPtr rgbData);
};

template<class PointT>
VideoSource<PointT>::~VideoSource() {
  // Empty!
}

template<class PointT>
void VideoSource<PointT>::setNextFrame(FrameDataPtr frameData) 
{
  FrameDataSubject::notifyObservers(frameData);
}

template<class PointT>
void VideoSource<PointT>::setNextFrame(RGBDataPtr rgbData) 
{
  RGBDataSubject::notifyObservers(rgbData);
}


}  // namespace lepp

#endif
