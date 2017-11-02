#ifndef OFFLINE_VIDEO_SOURCE_H_
#define OFFLINE_VIDEO_SOURCE_H_

#include <memory>

#include <opencv2/opencv.hpp>
#include <pcl/io/grabber.h>

#include "lepp3/FrameData.hpp"
#include "lepp3/RGBData.hpp"
#include "lepp3/VideoSource.hpp"


namespace lepp {

/**
 * A convenience class for an offline stream captured from a sequence of files.
 *
 * This class is in direct connection with lepp::VideoRecorder, where the input
 * is recorded in a customzied way (sequence of point clouds, RGB Images and a
 * file containing all kinematics.)
 */
template<class PointT>
class OfflineVideoSource : public VideoSource<PointT> {
public:
  OfflineVideoSource(boost::shared_ptr<pcl::Grabber> pcd_interface,
                     boost::shared_ptr<cv::VideoCapture> rgb_interface,
                     std::shared_ptr<lepp::PoseService> pose_service);

  virtual ~OfflineVideoSource();

  virtual void open();

  virtual void setOptions(const std::map<std::string, bool>& options) {}

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
  const boost::shared_ptr<pcl::Grabber> pcd_interface_;
  const boost::shared_ptr<cv::VideoCapture> rgb_interface_;

  long frameCount;
};

template<class PointT>
OfflineVideoSource<PointT>::OfflineVideoSource(
    boost::shared_ptr<pcl::Grabber> pcd_interface,
    boost::shared_ptr<cv::VideoCapture> rgb_interface,
    std::shared_ptr<PoseService> pose_service)
    : VideoSource<PointT>(pose_service),
      pcd_interface_(pcd_interface),
      rgb_interface_(rgb_interface),
      frameCount(0) {
}

template<class PointT>
OfflineVideoSource<PointT>::~OfflineVideoSource() {
  pcd_interface_->stop();

  if (rgb_interface_ && rgb_interface_->isOpened())
    rgb_interface_->release();
}

template<class PointT>
void OfflineVideoSource<PointT>::open() {
  // Register the callback and start grabbing frames...
  typedef void (callback_t)(const PointCloudConstPtr&);
  boost::function<callback_t> f = boost::bind(
      &OfflineVideoSource::cloud_cb_, this, _1);
  pcd_interface_->registerCallback(f);
  pcd_interface_->start();
}

template<class PointT>
void OfflineVideoSource<PointT>::cloud_cb_(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud) {
#ifdef LEPP3_ENABLE_TRACING
  tracepoint(lepp3_trace_provider, new_depth_frame);
#endif

  // Cloud
  FrameDataPtr frameData(new FrameData(++frameCount));
  frameData->cloud = cloud;
  this->setNextFrame(frameData);

  // RGB image
  if (rgb_interface_) {
#ifdef LEPP3_ENABLE_TRACING
    tracepoint(lepp3_trace_provider, new_rgb_frame);
#endif
    if (!rgb_interface_->grab()) {
      // We reached the end, wrap around
      rgb_interface_->set(CV_CAP_PROP_POS_FRAMES, 0);
      rgb_interface_->grab();
    }

    cv::Mat image;
    rgb_interface_->retrieve(image);

    RGBDataPtr rgbData(new RGBData(frameCount, image));
    this->setNextFrame(rgbData);
  }
}

}

#endif
