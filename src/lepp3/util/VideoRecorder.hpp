#ifndef LEPP3_VIDEO_RECORDER_H_
#define LEPP3_VIDEO_RECORDER_H_

#include <ctime>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "lepp3/FrameData.hpp"
#include "lepp3/RGBData.hpp"

#include "lepp3/util/util.h"
#include "lepp3/debug/timer.hpp"


namespace {
  std::string get_dir(std::string const& base_dir) {
    std::stringstream ss;
    ss << base_dir;
    if ('/' != base_dir[base_dir.size() - 1]
        && '/' != base_dir[base_dir.size() - 1]) {
      ss << '/';
    }
    ss << "rec_" << lepp::get_current_timestamp();
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
class VideoRecorder : public FrameDataObserver, public RGBDataObserver {
public:
  VideoRecorder(std::string const& outputPath);
  /**
   * Implementation of the FrameDataObserver interface.
  **/
  virtual void updateFrame(FrameDataPtr frameData);

  /**
   * Implementation of the RGBDataObserver interface.
  **/
  virtual void updateFrame(RGBDataPtr rgbData);

  /**
   * Set different recording options.
   * Determines whether to record the point cloud, rgb image and pose.
   */
  void setMode(bool cloud, bool rgb, bool pose);
private:
  /**
   * Write the current point cloud on disk.
   */
  void savePointCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);
  /**
   * Write the current pose parameters in file.
   */
  void saveParams(lepp::LolaKinematicsParams const& params);
  /**
   * Save the current image on disk.
   */
  void saveImage(cv::Mat const& image);
  /**
   * Recording options
   */
  bool record_cloud_, record_rgb_, record_pose_;
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
  bool cloud_lk_;
};

template<class PointT>
VideoRecorder<PointT>::VideoRecorder(std::string const& outputPath)
  : path_( get_dir(outputPath) ),
    params_file_name_("params.txt"),
    record_cloud_(true),
    record_rgb_(false),
    record_pose_(false),
    cloud_idx_(-1),
    image_idx_(-1),
    params_idx_(-1),
    cloud_lk_(false) {

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
void VideoRecorder<PointT>::updateFrame(FrameDataPtr frameData)
{
  // Save the point cloud if
  // 1. we are actually told to save it,
  // 2. there is no previous cloud waiting for the completion of the recording
  //    chain.
  if (cloud_lk_) {
    return;
  }

  if (record_cloud_) {
    ++cloud_idx_;
    savePointCloud(frameData->cloud);
    // Set the cloud lock only if here is not the end of recording chain (if
    // either rgb or pose is also going to be recorded)

    if (record_rgb_)
      cloud_lk_ = true;
  }


  if (record_pose_)
  {
    ++params_idx_;

    LolaKinematicsParams params(*frameData->lolaKinematics);
    params.frame_num = params_idx_;
    saveParams(params);
    if (record_rgb_)
      cloud_lk_ = true;
  }
}

template<class PointT>
void VideoRecorder<PointT>::updateFrame(RGBDataPtr rgbData) {
  // Exception: Make the cloud lock ineffective, if we are not recording any
  // point clouds.
  if (!record_cloud_ && !record_pose_)
    cloud_lk_ = true;

  // Save the image if
  // 1. we are actually told to save it,
  // 2. there is already a [supposedly] saved point cloud.
  // 3. there is no previous image waiting for the completion of the recording
  //    chain.
  if (record_rgb_) {
    if(cloud_lk_) {
      ++image_idx_;
      saveImage(rgbData->image);

      // here is the end of recording chain. release all
      // remaining locks.
      cloud_lk_ = false;
    }
  }
}

template<class PointT>
void VideoRecorder<PointT>::savePointCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud) {
  std::stringstream file_name;
  file_name << "cloud_" << std::setfill('0') << std::setw(4) << cloud_idx_ << ".pcd";

  Timer t;
  t.start();
  pcl::io::savePCDFileBinary (file_name.str(), *cloud);
  t.stop();
  // ++counter_;
  std::cout<<"SAVING CLOUD TOOK: " << t.duration() << " ms" << std::endl;
}

template<class PointT>
void VideoRecorder<PointT>::saveImage(cv::Mat const& image) {
  Timer t;
  t.start();

  std::stringstream file_name;
  file_name << "image_" << std::setfill('0') << std::setw(4) << image_idx_ << ".jpg";

  cv::imwrite(file_name.str(), image);
  t.stop();
  std::cout<<"SAVING IMAGE TOOK: " << t.duration() << " ms" << std::endl;
}

template<class PointT>
void VideoRecorder<PointT>::saveParams(LolaKinematicsParams const& params) {
  Timer t;
  t.start();
  std::ofstream tf_fout_;
  // open the file and add the current params to the end of it
  tf_fout_.open(params_file_name_.c_str(), std::ofstream::app);
  // write the parameters to the file
  for (size_t i = 0; i < 3; ++i) {
    tf_fout_ << params.t_wr_cl[i] << "\t";
  }
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      tf_fout_ << params.R_wr_cl[i][j] << "\t";
    }
  }
  for (size_t i = 0; i < 3; ++i) {
    tf_fout_ << params.t_stance_odo[i] << "\t";
  }
  tf_fout_ << params.phi_z_odo << "\t";
  tf_fout_ << params.stance << "\t";
  tf_fout_ << params.frame_num << "\t";
  tf_fout_ << params.stamp;
  tf_fout_ << std::endl;
  // ... and close the file.
  tf_fout_.close();
  t.stop();
  std::cout << "SAVING PARAMS TOOK: " << t.duration() << " ms" << std::endl;
}

#endif // LEPP3_VIDEO_RECORDER_H_
