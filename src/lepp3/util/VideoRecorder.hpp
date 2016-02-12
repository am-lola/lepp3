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
    : path_( get_dir_name() ),
      tf_file_name_("tf.txt"),
      cloud_lk_(false),
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

  boost::filesystem::path path_;
  std::string tf_file_name_;
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
void VideoRecorder<PointT>::notifyNewTF(int idx, const LolaKinematicsParams& params) {
  tf_idx_ = idx;
  std::cout << "Recorder::tf_idx_ : " << idx << std::endl;

  Timer t;
  t.start();
  // open the file
  std::ofstream tf_fout_;
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
  std::cout << "SAVING POSE TOOK: " << t.duration() << " ms" << std::endl;
}

#endif // LEPP3_VIDEO_RECORDER_H_
