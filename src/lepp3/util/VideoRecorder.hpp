#ifndef VIDEO_RECORDER_H_
#define VIDEO_RECORDER_H_

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <ctime>
#include <vector>
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>

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
} // namespace
/**
  * A recorder module that receives a point cloud and a pose (LolaKinematics)
  * from the robot and saves them alongside each other for further offline use.
  */
template<class PointT>
class VideoRecorder : public VideoObserver<PointT>, public TFObserver {
public:
  VideoRecorder()
    : p_( get_dir_name() ),
      cloud_lk_(false),
      tf_lk_(false) {
      if(boost::filesystem::exists(p_)) {
        if(boost::filesystem::is_directory(p_))
          std::cout << "directory " << p_ << "already exists. terminating...\n";
          // TODO finish up the termination process
      }
      else {
        boost::filesystem::create_directory(p_);
        //change current path to the new directory
        boost::filesystem::current_path(p_);
      }
    }
  /**
   * Implementation of the VideoObserver interface.
  **/
  virtual void notifyNewFrame(
      int idx,
      const typename pcl::PointCloud<PointT>::ConstPtr& cloud);
  /**
   * Implementation of the PoseObserver interface.
  **/
  virtual void notifyNewTF(int idx, const LolaKinematicsParams& tf);
private:

  boost::filesystem::path p_;
  int cloud_idx_;
  int tf_idx_;
  bool cloud_lk_, tf_lk_;
};

template<class PointT>
void VideoRecorder<PointT>::notifyNewFrame(
    int idx,
  const typename pcl::PointCloud<PointT>::ConstPtr& cloud) {
    // ADD the cloud to a buffer with corresponding index
    // process the buffer to save the cloud into file
    cloud_idx_ = idx;
    std::cout << "Recorder::cloud_idx_ : " << idx << std::endl;
    std::stringstream ss;
    ss << "cloud_" << cloud_idx_ << ".pcd";
    const std::string file_name = ss.str();
    Timer t;
    t.start();
    pcl::io::savePCDFileBinary (file_name, *cloud);
    t.stop();
    // ++counter_;
    std::cout<<"SAVING CLOUD TOOK: " << t.duration() << " ms" << std::endl;
}

template<class PointT>
void VideoRecorder<PointT>::notifyNewTF(int idx, const LolaKinematicsParams& tf) {
  tf_idx_ = idx;
  std::cout << "Recorder::tf_idx_ : " << idx << std::endl;

  Timer t;
  t.start();
  std::stringstream tf_ss;
  tf_ss << "tf_" << tf_idx_ << ".txt";
  const std::string& tmp = tf_ss.str();
  const char* name = tmp.c_str();
  std::ofstream tf_file;
  tf_file.open(name);
  tf_file << tf;
  tf_file.close();
  t.stop();
  std::cout << "SAVING POSE TOOK: " << t.duration() << " ms" << std::endl;
}

#endif // VIDEO_RECORDER_H_
