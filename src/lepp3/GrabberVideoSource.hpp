#ifndef GRABBER_VIDEO_SOURCE_H_
#define GRABBER_VIDEO_SOURCE_H_

#include "BaseVideoSource.hpp"
#include <pcl/io/openni_grabber.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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
        rgb_viewer_enabled_(false) {}
  GeneralGrabberVideoSource(boost::shared_ptr<pcl::Grabber> interface, bool rgb_enabled)
      : interface_(interface),
        rgb_viewer_enabled_(rgb_enabled) {}
  virtual ~GeneralGrabberVideoSource();
  virtual void open();
  bool rgb_viewer_enabled_;
private:
  /**
   * A reference to the Grabber instance that the VideoSource wraps.
   */
  const boost::shared_ptr<pcl::Grabber> interface_;

  /**
   * Member function which is registered as a callback of the Grabber.
   * Acts as the bond between the VideoSource and the Grabber, allowing the
   * adaptation of the interface.
   */
  void cloud_cb_(const typename pcl::PointCloud<PointT>::ConstPtr& cloud);
  void image_cb_ (const boost::shared_ptr<openni_wrapper::Image>& rgb);
};

template<class PointT>
GeneralGrabberVideoSource<PointT>::~GeneralGrabberVideoSource() {
  // RAII: make sure to stop any running Grabber
  interface_->stop();
}

template<class PointT>
void GeneralGrabberVideoSource<PointT>::cloud_cb_(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud) {
  this->setNextFrame(cloud);
}

template<class PointT>
void GeneralGrabberVideoSource<PointT>::image_cb_ (
    const typename boost::shared_ptr<openni_wrapper::Image>& rgb) {
  cv::Mat frameRGB = cv::Mat(rgb->getHeight(), rgb->getWidth(), CV_8UC3);
  rgb->fillRGB(frameRGB.cols,frameRGB.rows,frameRGB.data,frameRGB.step);
  cv::Mat frameBGR;
  cv::cvtColor(frameRGB,frameBGR,CV_RGB2BGR);

  cv::Mat frame = frameBGR;


  imshow( "RGB CAM", frame );
  cv::waitKey(30);
}


template<class PointT>
void GeneralGrabberVideoSource<PointT>::open() {
  // Register the callback and start grabbing frames...
  typedef void (callback_t)(const typename pcl::PointCloud<PointT>::ConstPtr&);
  boost::function<callback_t> f = boost::bind(
      &GeneralGrabberVideoSource::cloud_cb_,
      this, _1);
  interface_->registerCallback(f);

  if(rgb_viewer_enabled_) {
    boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> g =
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
  LiveStreamSource()
      : GeneralGrabberVideoSource<PointT>(boost::shared_ptr<pcl::Grabber>(
            new pcl::OpenNIGrabber("", pcl::OpenNIGrabber::OpenNI_QVGA_30Hz))) {
    // Empty... All work performed in the initializer list.
  }
  LiveStreamSource(bool rgb_enabled)
      : GeneralGrabberVideoSource<PointT>(boost::shared_ptr<pcl::Grabber>(
            new pcl::OpenNIGrabber("", pcl::OpenNIGrabber::OpenNI_QVGA_30Hz)),
            rgb_enabled) {
    // Empty... All work performed in the initializer list.
  }

};

}  // namespace lepp

#endif
