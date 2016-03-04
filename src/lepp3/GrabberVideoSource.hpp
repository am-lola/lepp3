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
  GeneralGrabberVideoSource(
    boost::shared_ptr<pcl::Grabber> interface,
    bool rgb_online = false);
  GeneralGrabberVideoSource(
    boost::shared_ptr<pcl::Grabber> interface,
    boost::shared_ptr<cv::VideoCapture> vc);
  virtual ~GeneralGrabberVideoSource();
  virtual void open();
  bool online_rgb_;
private:
  /**
   * A reference to the Grabber instance that the VideoSource wraps.
   */
  const boost::shared_ptr<pcl::Grabber> interface_;
  const boost::shared_ptr<cv::VideoCapture> offline_vc_;
  /**
   * Member function which is registered as a callback of the Grabber.
   * Acts as the bond between the VideoSource and the Grabber, allowing the
   * adaptation of the interface.
   */
  void cloud_cb_(const typename pcl::PointCloud<PointT>::ConstPtr& cloud);
  void image_cb_ (const boost::shared_ptr<openni_wrapper::Image>& rgb);
};

template<class PointT>
GeneralGrabberVideoSource<PointT>::GeneralGrabberVideoSource(
  boost::shared_ptr<pcl::Grabber> interface,
  bool rgb_online)
    : interface_(interface),
      online_rgb_(rgb_online) {}

template<class PointT>
GeneralGrabberVideoSource<PointT>::GeneralGrabberVideoSource(
  boost::shared_ptr<pcl::Grabber> interface,
  boost::shared_ptr<cv::VideoCapture> vc)
    : interface_(interface),
      offline_vc_(vc) {}

template<class PointT>
GeneralGrabberVideoSource<PointT>::~GeneralGrabberVideoSource() {
  // RAII: make sure to stop any running Grabber
  interface_->stop();
  if (offline_vc_->isOpened())
    offline_vc_->release();
}

template<class PointT>
void GeneralGrabberVideoSource<PointT>::cloud_cb_(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud) {
  std::cout << "entered GeneralGrabberVideoSource::cloud_cb_" << std::endl;

  this->setNextFrame(cloud);

  // If in offline mode, read the image sequence
  if (offline_vc_ != NULL) {
    cv::Mat image;
    offline_vc_->read(image);
    if (image.empty())
      std::cerr << "received empty image!" << std::endl;
    this->setNextFrame(image);
  }
}

template<class PointT>
void GeneralGrabberVideoSource<PointT>::image_cb_ (
    const typename boost::shared_ptr<openni_wrapper::Image>& rgb) {
  this->setNextFrame(rgb);
}


template<class PointT>
void GeneralGrabberVideoSource<PointT>::open() {
  // Register the callback and start grabbing frames...
  typedef void (callback_t)(const typename pcl::PointCloud<PointT>::ConstPtr&);
  boost::function<callback_t> f = boost::bind(
      &GeneralGrabberVideoSource::cloud_cb_,
      this, _1);
  interface_->registerCallback(f);

  if(online_rgb_) {
    std::cout << "setting opeenni_grabber's image_callback" << std::endl;
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
