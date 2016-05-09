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
  GeneralGrabberVideoSource(boost::shared_ptr<pcl::Grabber> interface);
  virtual ~GeneralGrabberVideoSource();
  virtual void open();
  /**
   * Implementation of VideoSource interface
   */
  virtual void setOptions(std::map<std::string, bool> options);
protected:
  /**
   * Member function which is registered as a callback of the Grabber.
   * Acts as the bond between the VideoSource and the Grabber, allowing the
   * adaptation of the interface.
   */
  virtual void cloud_cb_(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud);
private:
  /**
   * A reference to the Grabber instance that the VideoSource wraps.
   */
  const boost::shared_ptr<pcl::Grabber> interface_;
  void image_cb_ (const boost::shared_ptr<openni_wrapper::Image>& rgb);
  /**
   * Boolean values which determine whether the interface subsribes to receive
   * the point cloud and/or image.
   */
  bool receive_cloud_, receive_image_;
};

template<class PointT>
GeneralGrabberVideoSource<PointT>::GeneralGrabberVideoSource(
  boost::shared_ptr<pcl::Grabber> interface)
    : interface_(interface),
      receive_cloud_(true),
      receive_image_(false) {}

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
  this->setNextFrame(rgb);
}

template<class PointT>
void GeneralGrabberVideoSource<PointT>::setOptions(
    std::map<std::string, bool> options) {
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
  if (receive_cloud_) {
    typedef void (callback_t)(const typename pcl::PointCloud<PointT>::ConstPtr&);
    boost::function<callback_t> f = boost::bind(
        &GeneralGrabberVideoSource::cloud_cb_,
        this, _1);
    interface_->registerCallback(f);
  }

  if (receive_image_) {
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
};

/**
 * A convenience class for an offline stream captured from a sequence of files.
 *
 * This class is in direct connection with lepp::VideoRecorder, where the input
 * is recorded in a customzied way (sequence of point clouds, RGB Images and a
 * file containing all kinematics.)
 */
template<class PointT>
class OfflineVideoSource : public GeneralGrabberVideoSource<PointT> {
public:
  OfflineVideoSource(
    boost::shared_ptr<pcl::Grabber> pcd_interface,
    boost::shared_ptr<cv::VideoCapture> vc);
  virtual ~OfflineVideoSource();
protected:
  /**
   * Implementation of GeneralGrabberVideoSource::cloud_cb_ with the focus on
   * the RGB image sequence as well as the incoming point cloud.
   */
  void cloud_cb_(const typename pcl::PointCloud<PointT>::ConstPtr& cloud);
private:
  /**
   * cv::VideoCapture instance that is responsible for reading the rgb image
   * sequence.
   */
  const boost::shared_ptr<cv::VideoCapture> rgb_interface_;

};

template<class PointT>
OfflineVideoSource<PointT>::OfflineVideoSource(
  boost::shared_ptr<pcl::Grabber> pcd_interface,
  boost::shared_ptr<cv::VideoCapture> vc)
    : GeneralGrabberVideoSource<PointT>(pcd_interface),
      rgb_interface_(vc) {
  // Subscription to cloud and image (a.k.a internal receive_cloud_ and
  // receive_image_) is already taken care of by default in
  // GeneralGrabberVideoSource::setOptions
}

template<class PointT>
OfflineVideoSource<PointT>::~OfflineVideoSource() {
  // RAII: make sure to stop any running Grabber
  // destructors are called automatically in the reverse order of construction
  if (rgb_interface_->isOpened())
    rgb_interface_->release();
}

template<class PointT>
void OfflineVideoSource<PointT>::cloud_cb_(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud) {

  this->setNextFrame(cloud);

  // Read the next RGB image in the sequence along with current point cloud.
  if (rgb_interface_ != NULL) {
    cv::Mat image;
    rgb_interface_->read(image);
    // Check if reached the end of image sequence.
    if(image.empty()) {
      // seek back to the first frame
      rgb_interface_->set(CV_CAP_PROP_POS_FRAMES, 0);
      // Read the frame (because the point cloud is already there).
      rgb_interface_->read(image);
    }
    this->setNextFrame(image);
  }
}

}  // namespace lepp

#endif
