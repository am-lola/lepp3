#ifndef LEPP3_VIDEO_RECORDER_H_
#define LEPP3_VIDEO_RECORDER_H_

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <ctime>
#include <vector>
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "lola/TransformObserver.hpp"
#include "lepp3/VideoObserver.hpp"

#include "lepp3/debug/timer.hpp"


namespace {
/**
* generate a name based on current timestamp which will be used to create a
* directory.
**/
std::string get_dir_name() {
  std::stringstream ss;
  time_t t = time(0);   // get time now
  struct tm * now = localtime( & t );
  ss << (now->tm_year + 1900) << '-'
      << (now->tm_mon + 1) << '-'
      <<  now->tm_mday << '_'
      << now->tm_hour << now->tm_min << now->tm_sec;
  return ss.str();
}

std::ostream& operator<<(std::ostream& out, LolaKinematicsParams const& param) {
  out << "stamp #" << param.stamp << std::endl
      << "phi_z_odo = " << param.phi_z_odo << std::endl
      << "stance = " << param.stance << std::endl;
  out << "t_wr_cl = ";
  for (int i = 0; i < 3; ++i) out << param.t_wr_cl[i] << " "; out << std::endl;

  out << "R_Wr_cl = " << std::endl;
  for (int i = 0; i < 3; ++i) {
    out << "  ";
    for (int j = 0; j < 3; ++j) {
      out << param.R_wr_cl[i][j] << " ";
    }
    out << std::endl;
  }
  out << "t_stance_odo = ";
  for (int i = 0; i < 3; ++i) out << param.t_stance_odo[i] << " "; out << std::endl;

  return out;
}

} // namespace anonymous

namespace lepp{
  enum RecordingMode {
    MODE_CLOUD = 0,
    MODE_CLOUD_IMAGE,
    MODE_CLOUD_IMAGE_POSE
  };
} // namespace lepp

/**
  * A recorder module that receives a point cloud and a pose (LolaKinematics)
  * from the robot and saves them alongside each other for further offline use.
  */
template<class PointT>
class VideoRecorder : public VideoObserver<PointT>, public TFObserver {
public:
  VideoRecorder();
  /**
   * Implementation of the VideoObserver interface.
  **/
  virtual void notifyNewFrame(
      int idx,
      const typename pcl::PointCloud<PointT>::ConstPtr& cloud);
  /**
   * Implementation of the VideoObserver interface.
  **/
  virtual void notifyNewFrame(
      int idx,
      const typename boost::shared_ptr<openni_wrapper::Image>& image);
  void notifyNewFrame(int idx, const cv::Mat& image) {};
  /**
   * Implementation of the PoseObserver interface.
  **/
  virtual void notifyNewTF(int idx, const LolaKinematicsParams& params);
  void setMode(lepp::RecordingMode mode);
private:
  void savePointCloud();
  void writeParams(const LolaKinematicsParams& params);
  void saveImage();
  lepp::RecordingMode rec_mode_;
  typename pcl::PointCloud<PointT>::ConstPtr cloud_;
  boost::shared_ptr<openni_wrapper::Image> image_;
  boost::filesystem::path path_;
  std::string tf_file_name_;
  int cloud_idx_;
  int image_idx_;
  int tf_idx_;
  bool cloud_lk_, image_lk_, tf_lk_;
};

template<class PointT>
VideoRecorder<PointT>::VideoRecorder()
  : path_( get_dir_name() ),
    tf_file_name_("tf.txt"),
    rec_mode_(MODE_CLOUD_IMAGE),
    cloud_idx_(-1),
    image_idx_(-1),
    tf_idx_(-1),
    cloud_lk_(false),
    image_lk_(false),
    tf_lk_(false) {

  if(boost::filesystem::exists(path_)) {
    if(boost::filesystem::is_directory(path_))
      std::cout << "directory " << path_ << "already exists. terminating...\n";
      // TODO finish up the termination process
  }
  else {
    boost::filesystem::create_directory(path_);
    //change current path to the new directory
    boost::filesystem::current_path(path_);

    // write header to tf_file
    std::ofstream tf_fout(tf_file_name_.c_str());
    if(tf_fout.is_open()) {
      tf_fout << "# t_wr_cl[0],\tt_wr_cl[1],\tt_wr_cl[2],\t"
              << "R_wr_cl[0][0],\tR_wr_cl[0][1],\tR_wr_cl[0][2],\t"
              << "R_wr_cl[1][0],\tR_wr_cl[1][1],\tR_wr_cl[1][2],\t"
              << "R_wr_cl[2][0],\tR_wr_cl[2][1],\tR_wr_cl[2][2],\t"
              << "t_stance_odo[0],\tt_stance_odo[1],\tt_stance_odo[2],\t"
              << "phi_z_odo,\tstance,\tframe_num,\tstamp"
              << std::endl;
      tf_fout.close();
    }
  }
}

template<class PointT>
void VideoRecorder<PointT>::setMode(lepp::RecordingMode mode) {
  rec_mode_ = mode;
}

template<class PointT>
void VideoRecorder<PointT>::notifyNewFrame(
    int idx,
  const typename pcl::PointCloud<PointT>::ConstPtr& cloud) {
  // get the cloud and lock
  if(!cloud_lk_) {
    cloud_ = cloud;
    cloud_idx_++;
    cloud_lk_ = true;
  }
  else
    std::cout << "cloud reader is still locked." << std::endl;
}

template<class PointT>
void VideoRecorder<PointT>::notifyNewFrame(
    int idx,
    const typename boost::shared_ptr<openni_wrapper::Image>& image) {

  std::cout << "entered image callback" << std::endl;
  if(cloud_lk_) {
    image_idx_++;
    image_ = image;
    if(rec_mode_ = MODE_CLOUD_IMAGE) {
      savePointCloud();
      saveImage();
      cloud_lk_ = false;
      image_lk_ = false;
      std::cout << "releasing cloud lock" << std::endl;
      return;
    }
    image_lk_ = true;
  }

}

template<class PointT>
void VideoRecorder<PointT>::notifyNewTF(int idx, const LolaKinematicsParams& params) {
  tf_idx_ = idx;
  std::cout << "Recorder::tf_idx_ : " << idx << std::endl;

  if (cloud_lk_) {
    tf_idx_++;
    // first, write the point cloud to disk
    savePointCloud();
    writeParams(params);
    saveImage();
    cloud_lk_ = false;
    image_lk_ = false;
  }
}

template<class PointT>
void VideoRecorder<PointT>::savePointCloud() {
  std::stringstream ss;
  ss << "cloud_" << cloud_idx_ << ".pcd";
  const std::string file_name = ss.str();
  Timer t;
  t.start();
  pcl::io::savePCDFileBinary (file_name, *cloud_);
  t.stop();
  // ++counter_;
  std::cout<<"SAVING CLOUD TOOK: " << t.duration() << " ms" << std::endl;
}

template<class PointT>
void VideoRecorder<PointT>::saveImage() {
  Timer t;
  t.start();
  cv::Mat frameRGB = cv::Mat(image_->getHeight(), image_->getWidth(), CV_8UC3);
  image_->fillRGB(frameRGB.cols,frameRGB.rows,frameRGB.data,frameRGB.step);

  std::stringstream ss;
  if (0 <= image_idx_ && image_idx_< 10)
    ss << "image_000" << image_idx_ << ".jpg";
  if (10 <= image_idx_ && image_idx_< 100)
    ss << "image_00" << image_idx_ << ".jpg";
  if (100 <= image_idx_ && image_idx_< 1000)
    ss << "image_0" << image_idx_ << ".jpg";
  if (1000 <= image_idx_ && image_idx_< 10000)
    ss << "image_" << image_idx_ << ".jpg";
  if (image_idx_ > 10000) {
    std::cout << "reached max size for saving RGB images.";
    return;
  }
  const std::string file_name = ss.str();

  cv::imwrite(file_name, frameRGB);
  t.stop();
  std::cout<<"SAVING IMAGE TOOK: " << t.duration() << " ms" << std::endl;
}

template<class PointT>
void VideoRecorder<PointT>::writeParams(const LolaKinematicsParams& params) {
  Timer t;
  t.start();
  // open the file
  std::ofstream tf_fout_;
  // open the file and add the current params to the end of it
  tf_fout_.open(tf_file_name_.c_str(), std::ofstream::app);
  // write the parameters to the file
  std::stringstream ss;
  for (size_t i = 0; i < 3; ++i) { ss << params.t_wr_cl[i] << "\t"; }
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      ss << params.R_wr_cl[i][j] << "\t";
    }
  }
  for (size_t i = 0; i < 3; ++i) { ss << params.t_stance_odo[i] << "\t"; }
  ss << params.phi_z_odo << "\t";
  ss << params.stance << "\t";
  ss << params.frame_num << "\t";
  ss << params.stamp;
  tf_fout_ << ss.str() << std::endl;
  // and close the file...
  tf_fout_.close();
  t.stop();
  std::cout << "SAVING PARAMS TOOK: " << t.duration() << " ms" << std::endl;
}

#endif // LEPP3_VIDEO_RECORDER_H_
