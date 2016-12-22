#ifndef GRABBER_VIDEO_SOURCE_H_
#define GRABBER_VIDEO_SOURCE_H_

#include "lepp3/Typedefs.hpp"
#include "lepp3/FrameData.hpp"
#include "lepp3/RGBData.hpp"
#include "VideoSource.hpp"

#include <pcl/io/openni2_grabber.h>

namespace lepp {

/**
 * An implementation of the VideoSource abstract base class, based on PCL's
 * Grabber class.
 *
 * It allows clients to wrap a PCL Grabber to the VideoSource interface.
 * The grabber which is wrapped needs to be injected at construction of the
 * source instance.
 */
template<class PointT>
class GeneralGrabberVideoSource : public VideoSource<PointT> {
public:
  /**
   * Instantiate a video source which wraps the given Grabber instance.
   * The VideoSource instance takes ownership of the given Grabber instance.
   */
  GeneralGrabberVideoSource(boost::shared_ptr<pcl::Grabber> interface)
      : interface_(interface),
        rgb_viewer_enabled_(false),
        frameCount(0),
        receive_cloud_(true),
        receive_image_(false) {}
  GeneralGrabberVideoSource(boost::shared_ptr<pcl::Grabber> interface, bool rgb_enabled)
      : interface_(interface),
        rgb_viewer_enabled_(rgb_enabled),
        receive_cloud_(true),
        receive_image_(rgb_enabled),
        frameCount(0) {}

  virtual ~GeneralGrabberVideoSource();
  virtual void open();
  bool rgb_viewer_enabled_;
  /**
   * Implementation of VideoSource interface
   */
  virtual void setOptions(const std::map<std::string, bool>& options) override;

protected:
  /**
  * A reference to the Grabber instance that the VideoSource wraps.
  */
  const boost::shared_ptr<pcl::Grabber> interface_;

  /**
   * Member function which is registered as a callback of the Grabber.
   * Acts as the bond between the VideoSource and the Grabber, allowing the
   * adaptation of the interface.
   */
  void cloud_cb_(const PointCloudConstPtr& cloud);
  void image_cb_ (const boost::shared_ptr<pcl::io::Image>& rgb);

  long frameCount;
  /**
   * Boolean values which determine whether the interface subsribes to receive
   * the point cloud and/or image.
   */
  bool receive_cloud_, receive_image_;
};

template<class PointT>
GeneralGrabberVideoSource<PointT>::~GeneralGrabberVideoSource() {
  // RAII: make sure to stop any running Grabber
  interface_->stop();
}

template<class PointT>
void GeneralGrabberVideoSource<PointT>::cloud_cb_(
    const PointCloudConstPtr& cloud)
{
  FrameDataPtr frameData(new FrameData(++frameCount));
  frameData->cloud = cloud;
  this->setNextFrame(frameData);
}

template<class PointT>
void GeneralGrabberVideoSource<PointT>::image_cb_ (
    const typename boost::shared_ptr<pcl::io::Image>& rgb)
{
  cv::Mat frameRGB = cv::Mat(rgb->getHeight(), rgb->getWidth(), CV_8UC3);
  rgb->fillRGB(frameRGB.cols, frameRGB.rows, frameRGB.data, frameRGB.step);

  RGBDataPtr rgbData(new RGBData(frameCount, frameRGB));
  this->setNextFrame(rgbData);
}

template<class PointT>
void GeneralGrabberVideoSource<PointT>::setOptions(
    const std::map<std::string, bool>& options) {
  for (std::pair<std::string, bool> const& key_value : options) {
    if (key_value.first == "subscribe_cloud") {
      receive_cloud_ = key_value.second;
    } else if (key_value.first == "subscribe_image") {
      receive_image_ = key_value.second;
    }
    else throw "Invalid grabber options.";
  }
}

template<class PointT>
void GeneralGrabberVideoSource<PointT>::open() {
  // Register the callback and start grabbing frames...
  if (receive_cloud_)
  {
    typedef void (callback_t)(const PointCloudConstPtr&);
    boost::function<callback_t> f = boost::bind(
        &GeneralGrabberVideoSource::cloud_cb_,
        this, _1);
    interface_->registerCallback(f);
  }

  if (receive_image_) {
    boost::function<void (const boost::shared_ptr<pcl::io::Image>&)> g =
         boost::bind (&GeneralGrabberVideoSource::image_cb_, this, _1);
    interface_->registerCallback(g);
  }

  interface_->start();
}

/**
 * A convenience class for a live stream captured from a local RGB-D sensor.
 *
 * The implementation leverages the GeneralGrabberVideoSource wrapping a
 * PCL-based OpenNIGrabber instance.
 */
template<class PointT>
class LiveStreamSource : public GeneralGrabberVideoSource<PointT> {
public:
  LiveStreamSource(bool enable_rgb = false)
      : GeneralGrabberVideoSource<PointT>(boost::shared_ptr<pcl::Grabber>(
            new pcl::io::OpenNI2Grabber()),
            enable_rgb
          ) {
    // Empty... All work performed in the initializer list.
  }
};

}  // namespace lepp

#endif
