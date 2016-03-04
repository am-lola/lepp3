#ifndef VIDEO_OBSERVER_H_
#define VIDEO_OBSERVER_H_

#include <opencv2/core/core.hpp>

namespace lepp {

/**
 * An ABC (interface) that classes that wish to observe a particular
 * VideoSource need to implement.
 */
template<class PointT>
class VideoObserver {
public:
  /**
   * Method that the observers need to implement in order to handle a new
   * incoming point cloud frame.  Each frame is represented by an integer
   * identifier (representing the number of the frame in the sequence of the
   *  source) and the point cloud extracted from that frame.
   */
  virtual void notifyNewFrame(
      int idx,
      const typename pcl::PointCloud<PointT>::ConstPtr& pointCloud) = 0;
  /**
   * Method that the observers need to implement in order to handle a new
   * incoming RGB frame.  Each frame is represented by an integer identifier
   * (representing the number of the frame in the sequence of the source)
   * and the RGB image extracted from that frame.
   */
  virtual void notifyNewFrame(
      int idx,
      const typename boost::shared_ptr<openni_wrapper::Image>& image) = 0;
  /**
   * Method that the observers need to implement in order to handle a new
   * incoming RGB frame in OFFLINE mode. Each frame is represented by an integer
   * identifier (representing the number of the frame in the sequence available
   * in the directory) and the corresponding RGB image.
   */
  virtual void notifyNewFrame(int idx, const cv::Mat& image) = 0;
};

}  // namespace lepp

#endif
