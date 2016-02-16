#ifndef LEPP3_VISUALIZATION_IMAGE_VISUALIZER_H__
#define LEPP3_VISUALIZATION_IMAGE_VISUALIZER_H__

#include "lepp3/VideoObserver.hpp"

namespace lepp {

template<class PointT>
class ImageVisualizer : public VideoObserver<PointT> {
public:
  ImageVisualizer() {
    cv::namedWindow("RGB CAM", cv::WINDOW_AUTOSIZE);
  }
  void notifyNewFrame(
      int idx,
      const typename pcl::PointCloud<PointT>::ConstPtr& pointCloud) {};
  void notifyNewFrame(
      int idx,
      const typename boost::shared_ptr<openni_wrapper::Image>& image) {};
  virtual void notifyNewFrame(int idx, const cv::Mat& image);
};

template<class PointT>
void ImageVisualizer<PointT>::notifyNewFrame(int idx, const cv::Mat& image) {
  std::cout << "showing RGB..." << std::endl;
  imshow("RGB CAM", image);
  cv::waitKey(30);
}

} // namespace lepp

#endif
