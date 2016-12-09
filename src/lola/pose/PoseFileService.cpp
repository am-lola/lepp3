#include "PoseFileService.hpp"

#include <fstream>
#include <sstream>
#include <string>

PoseFileService::PoseFileService(std::string const& filename)
    : pose_idx_(0) {
  std::ifstream param_file(filename);

  std::string line;
  while (std::getline(param_file, line)) {
    if ('#' == line[0]) {
      continue;
    }

    std::istringstream os(line);
    HR_Pose_Red pose;
    pose.version = 1;

    for (size_t i = 0; i < 3; ++i) {
      os >> pose.t_wr_cl[i];
    }
    for (size_t i = 0; i < 9; ++i) {
      os >> pose.R_wr_cl[i];
    }
    for (size_t i = 0; i < 3; ++i) {
      os >> pose.t_stance_odo[i];
    }
    os >> pose.phi_z_odo
       >> pose.stance
       >> pose.tick_counter
       >> pose.stamp;

    pose_data_.emplace_back(pose);
  }
}

HR_Pose_Red PoseFileService::getCurrentPose() const {
  size_t idx = pose_idx_ % pose_data_.size();
  return pose_data_[idx];
}

void PoseFileService::triggerNextFrame() {
  ++pose_idx_;

  LolaKinematicsParams params = getParams();
  notifyObservers(pose_idx_, params);
}