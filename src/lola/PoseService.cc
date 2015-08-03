#include "lola/PoseService.h"
#include "boost/thread.hpp"

#include <cstring>
#include <iostream>

#include "deps/easylogging++.h"

namespace {

/**
 * Puts a rotation matrix (around the z-axis) for the given angle in the given
 * matrix `matrix`.
 * It is assumed that the given matrix points to a matrix of dimensions 3x3.
 */
void rotationmatrix(double angle, double matrix[][3]) {
  double s = sin(angle);
  double c = cos(angle);

  matrix[0][0] = c; matrix[0][1] = -s; matrix[0][2] = 0;
  matrix[1][0] = s; matrix[1][1] = c; matrix[1][2] = 0;
  matrix[2][0] = 0; matrix[2][1] = 0; matrix[2][2] = 1;
}

/**
 * Transposes the given matrix `matrix` and puts the transpose result into the
 * given `transpose` matrix.
 *
 * The matrices are assumed to be 3x3.
 */
void transpose(double matrix[][3], double transpose[][3]) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      transpose[j][i] = matrix[i][j];
    }
  }
}

} // namespace anonymous

void PoseService::read_handler(
    boost::system::error_code const& ec,
    std::size_t bytes_transferred) {
  LINFO << "Pose Service: Received " << bytes_transferred;
  if (bytes_transferred != sizeof(HR_Pose)) {
    LERROR << "Pose Service: Error: Invalid datagram size."
           << "Expected " << sizeof(HR_Pose);
    // If this one fails, we still queue another receive...
    queue_recv();
    return;
  }
  boost::shared_ptr<HR_Pose> new_pose(new HR_Pose);
  // The copy is thread safe since nothing can be writing to the recv_buffer
  // at this point. No new async read is queued until this callback is complete.
  memcpy(&*new_pose, &recv_buffer_[0], sizeof(HR_Pose));
  // This performs an atomic update of the pointer, making it a lock-free,
  // thread-safe operation.
  pose_ = new_pose;
  LINFO << "Pose Service: Updated current pose";
  queue_recv();
}

void PoseService::service_thread() {
  LINFO << "Pose Service: Thread started";
  io_service_.run();
}

void PoseService::queue_recv() {
  socket_.async_receive(
      boost::asio::buffer(recv_buffer_),
      boost::bind(&PoseService::read_handler, this, _1, _2));
}

void PoseService::bind() {
  boost::system::error_code error;
  socket_.open(boost::asio::ip::udp::v4(), error);
  boost::asio::ip::udp::endpoint local(
      boost::asio::ip::address::from_string(host_),
      port_);
  socket_.bind(local);
}

void PoseService::start() {
  bind();
  queue_recv();
  // Start the thread that will run the associated I/O service.
  boost::thread(boost::bind(&PoseService::service_thread, this));
}


HR_Pose PoseService::getCurrentPose() const {
  // This does an atomic copy of the pointer (the refcount is atomically updated)
  // There can be no race condition since if the service needs to update the
  // pointer, it will do so atomically and the reader also obtains a copy of the
  // pointer atomically.
  boost::shared_ptr<HR_Pose> p = pose_;
  // Now we are safe to manipulate the object itself, since nothing else needs
  // to directly touch the instance itself and we have safely obtained a
  // reference to it.
  // We just return the object (but this involves a non-atomic copy, hence the
  // pointer dance).

  if (p) {
    return *p;
  } else {
    HR_Pose pose = {0};
    return pose;
  }
}

lepp::Coordinate PoseService::getRobotPosition() const {
  LolaKinematicsParams params = getParams();

  double rotation_matrix[3][3];
  rotationmatrix(params.phi_z_odo, rotation_matrix);

  // In pseudo-code (if matrix operations were supported):
  // ret = transpose(rotation_matrix) * (t_stance_odo)
  double transposed_matrix[3][3];
  transpose(rotation_matrix, transposed_matrix);
  std::vector<double> ret(3);
  for (int i = 0; i < 3; ++i) {
    ret[i] = 0;
    for (int j = 0; j < 3; ++j) {
      ret[i] +=
          transposed_matrix[i][j] * (params.t_stance_odo[j]);
    }
  }

  return lepp::Coordinate(ret[0], ret[1], ret[2]);
}

LolaKinematicsParams PoseService::getParams() const {
  HR_Pose pose = getCurrentPose();
  // Now convert the current raw pose to parameters that are of relevance to the
  // transformation.
  LolaKinematicsParams params;
  for (int i = 0; i < 3; ++i) {
    params.t_wr_cl[i] = pose.t_wr_cl[i];
    params.t_stance_odo[i] = pose.t_stance_odo[i];
    for (int j = 0; j < 3; ++j) {
      params.R_wr_cl[i][j] = pose.R_wr_cl[3*i + j];
    }
  }
  params.phi_z_odo = pose.phi_z_odo;
  params.stance = pose.stance;
  params.stamp = pose.stamp;

  return params;
}
