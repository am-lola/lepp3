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

#include "lola/PoseObserver.hpp"
#include "lepp3/VideoObserver.hpp"

#include "lepp3/util/util.h"
#include "lepp3/debug/timer.hpp"


namespace {
  std::string get_dir() {
    std::stringstream ss;
    ss << "../recordings/rec_" << lepp::get_current_timestamp();
    return ss.str();
  }
}
/**
  * A recorder module that receives a point cloud, an RGB image and a pose
  * (LolaKinematics) from the camera and the robot and saves them alongside
  * each other for further offline use.
  *
  * Since the publish rate of each of the above three elements are different,
  * a lock-and-wait mechanism is applied to avoid data inconsistency.
  *
  * As the point cloud publisher is believed to be slowest, and the pose Service
  * to be the fastest elements, a recording chain is assumed to save them
  * accordingly.
  *
  * The order of saving the elements are
  *     1. point cloud
  *     2. rgb image
  *     3. pose parameters
  * Regardless of how many incoming frames we encounter, the chain has to be
  * traversed completely each time, before saving a second instance of each
  * element. This ensures the time drift between them to be minimum.
  *
  * Recording each of these elements are totally optional and could be set by
  * the `setMode` method.
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
   */
  virtual void NotifyNewPose(int idx, const LolaKinematicsParams& params);
  /**
   * Set different recording options.
   * Determines whether to record the point cloud, rgb image and pose.
   */
  void setMode(bool cloud, bool rgb, bool pose);
private:
  /**
   * Write the current point cloud on disk.
   */
  void savePointCloud();
  /**
   * Write the current pose parameters in file.
   */
  void saveParams();
  /**
   * Save the current image on disk.
   */
  void saveImage();
  /**
   * Recording options
   */
  bool record_cloud_, record_rgb_, record_pose_;
  /**
   * Instance holding the current point cloud.
   */
  typename pcl::PointCloud<PointT>::ConstPtr cloud_;
  /**
   * Instance holding the current RGB image.
   */
  boost::shared_ptr<openni_wrapper::Image> image_;
  /**
   * Instance holding the current pose parameters.
   */
  LolaKinematicsParams params_;
  /**
   * System path to save the captured data.
   */
  boost::filesystem::path path_;
  /**
   * Name of the file which holds pose parameters.
   */
  std::string params_file_name_;
  /**
   * Internal indices to maintain the consistency between cloud, rgb and pose.
   */
  int cloud_idx_, image_idx_, params_idx_;
  /**
   *
   */
  bool cloud_lk_, image_lk_;
};

template<class PointT>
VideoRecorder<PointT>::VideoRecorder()
  : path_( get_dir() ),
    params_file_name_("params.txt"),
    record_cloud_(true),
    record_rgb_(false),
    record_pose_(false),
    cloud_idx_(-1),
    image_idx_(-1),
    params_idx_(-1),
    cloud_lk_(false),
    image_lk_(false) {

  namespace bfs = boost::filesystem;
  if(bfs::exists(path_)) {
    if(bfs::is_directory(path_))
      std::cout << "directory " << path_ << "already exists. terminating...\n";
      throw "No overwriting rule exists. Process terminated.";
  }
  else {
    bfs::create_directory(path_);
    // change current path to the new directory
    bfs::current_path(path_);

    // write header to tf_file
    std::ofstream tf_fout(params_file_name_.c_str());
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
void VideoRecorder<PointT>::setMode(bool cloud,
                                    bool rgb,
                                    bool pose) {
  record_cloud_ = cloud;
  record_rgb_   = rgb;
  record_pose_  = pose;
}

template<class PointT>
void VideoRecorder<PointT>::notifyNewFrame(
    int idx,
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud) {

  // Save the point cloud if
  // 1. we are actually told to save it,
  // 2. there is no previous cloud waiting for the completion of the recording
  //    chain.
  if (record_cloud_) {
    if (!cloud_lk_) {
      cloud_ = cloud;
      cloud_idx_++;
      savePointCloud();
      // Set the cloud lock only if here is not the end of recording chain (if
      // either rgb or pose is also going to be recorded)
      if (record_pose_ || record_rgb_)
        cloud_lk_ = true;
    }
  }
}

template<class PointT>
void VideoRecorder<PointT>::notifyNewFrame(
    int idx,
    const typename boost::shared_ptr<openni_wrapper::Image>& image) {

  std::cout << "IMAGE CALLBACK" << std::endl;
  // Exception: Make the cloud lock ineffective, if we are not recording any
  // point clouds.
  if (!record_cloud_)
    cloud_lk_ = true;

  // Save the image if
  // 1. we are actually told to save it,
  // 2. there is already a [supposedly] saved point cloud.
  // 3. there is no previous image waiting for the completion of the recording
  //    chain.
  if (record_rgb_) {
    if(cloud_lk_ && !image_lk_) {
      image_idx_++;
      image_ = image;
      saveImage();
      // set the image lock only if we are recording pose...
      if (record_pose_)
        image_lk_ = true;
      // ... otherwise, here is the end of recording chain. release all
      // remaining locks.
      else
        cloud_lk_ = false;
    }
  }
}

template<class PointT>
void VideoRecorder<PointT>::NotifyNewPose(
    int idx,
    const LolaKinematicsParams& params) {

  // Rare case: [DEBUG] Recorder only saves pose params. Make the cloud and
  // image locks ineffective.
  if (!record_cloud_ && !record_rgb_) {
    cloud_lk_ = true;
    image_lk_ = true;
  }

  // Save the parameters if
  // 1. we are actually told to save them,
  // 2. there is already a [supposedly] saved point cloud.
  // 3. there is already a [supposedly] saved image.
  if (record_pose_) {
    if (cloud_lk_ && image_lk_) {
      params_idx_++;
      params_ = params;
      // Modify the frame number to the internal counter
      params_.frame_num = params_idx_;
      saveParams();
      // Release the locks for next frames.
      cloud_lk_ = false;
      image_lk_ = false;
    }
  }
}

template<class PointT>
void VideoRecorder<PointT>::savePointCloud() {

  std::stringstream ss;
  if (0 <= cloud_idx_ && cloud_idx_< 10)
    ss << "cloud_000" << cloud_idx_ << ".pcd";
  if (10 <= cloud_idx_ && cloud_idx_< 100)
    ss << "cloud_00" << cloud_idx_ << ".pcd";
  if (100 <= cloud_idx_ && cloud_idx_< 1000)
    ss << "cloud_0" << cloud_idx_ << ".pcd";
  if (1000 <= cloud_idx_ && cloud_idx_< 10000)
    ss << "cloud_" << cloud_idx_ << ".pcd";
  if (image_idx_ > 10000) {
    throw "reached max size for saving point clouds!";
  }
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
    throw "reached max size for saving RGB images!";
  }
  const std::string file_name = ss.str();

  cv::imwrite(file_name, frameRGB);
  t.stop();
  std::cout<<"SAVING IMAGE TOOK: " << t.duration() << " ms" << std::endl;
}

template<class PointT>
void VideoRecorder<PointT>::saveParams() {
  Timer t;
  t.start();
  std::ofstream tf_fout_;
  // open the file and add the current params to the end of it
  tf_fout_.open(params_file_name_.c_str(), std::ofstream::app);
  // write the parameters to the file
  std::stringstream ss;
  for (size_t i = 0; i < 3; ++i) { ss << params_.t_wr_cl[i] << "\t"; }
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      ss << params_.R_wr_cl[i][j] << "\t";
    }
  }
  for (size_t i = 0; i < 3; ++i) { ss << params_.t_stance_odo[i] << "\t"; }
  ss << params_.phi_z_odo << "\t";
  ss << params_.stance << "\t";
  ss << params_.frame_num << "\t";
  ss << params_.stamp;
  tf_fout_ << ss.str() << std::endl;
  // ... and close the file.
  tf_fout_.close();
  t.stop();
  std::cout << "SAVING PARAMS TOOK: " << t.duration() << " ms" << std::endl;
}

#endif // LEPP3_VIDEO_RECORDER_H_
