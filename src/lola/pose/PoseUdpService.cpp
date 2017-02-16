#include "PoseUdpService.hpp"
#include <thread>
#include <boost/bind.hpp>
#include "deps/easylogging++.h"

PoseUdpService::~PoseUdpService() {
  io_service_.stop();
}

void PoseUdpService::read_handler(
    boost::system::error_code const& ec,
    std::size_t bytes_transferred) {
  LINFO << "Pose Service: Received " << bytes_transferred << " bytes";
  if (bytes_transferred != sizeof(HR_Pose_Red)) {
    LERROR << "Pose Service: Error: Invalid datagram size."
           << "Expected " << sizeof(HR_Pose_Red);
    // If this one fails, we still queue another receive...
    queue_recv();
    return;
  }

  std::shared_ptr<HR_Pose_Red> new_pose(new HR_Pose_Red);
  // The copy is thread safe since nothing can be writing to the recv_buffer
  // at this point. No new async read is queued until this callback is complete.
  memcpy(new_pose.get(), recv_buffer_.data(), sizeof(HR_Pose_Red));
  // This performs an atomic update of the pointer, making it a lock-free,
  // thread-safe operation
  received_pose_ = new_pose;
  LINFO << "Pose Service: Updated current pose";
  // notify any TFObserver of the new pose

  // Print parameters received
  LTRACE << "Received pose"
         << "  Phi_Z_ODO = " << new_pose->phi_z_odo
         << "  Stamp = " << new_pose->stamp
         << "  T_Stance_ODO.X = " << new_pose->t_stance_odo[0]
         << "  T_Stance_ODO.Y = " << new_pose->t_stance_odo[1]
         << "  T_Stance_ODO.Z = " << new_pose->t_stance_odo[2]
         << "  Version Nr. = " << new_pose->version
         << "  TIC counter = " << new_pose->tick_counter
         << "  Stance = " << static_cast<int>(new_pose->stance)
         << "  Size of HR_Pose = " << sizeof(HR_Pose_Red)
         << "  Size of Message = " << bytes_transferred
         << "  Translation.X= " << new_pose->t_wr_cl[0]
         << "  Translation.Y= " << new_pose->t_wr_cl[1]
         << "  Translation.Z= " << new_pose->t_wr_cl[2]
         << "  Rotation[0 0]= " << new_pose->R_wr_cl[0]
         << "  Rotation[0 1]= " << new_pose->R_wr_cl[1]
         << "  Rotation[0 2]= " << new_pose->R_wr_cl[2]
         << "  Rotation[1 0]= " << new_pose->R_wr_cl[3]
         << "  Rotation[1 1]= " << new_pose->R_wr_cl[4]
         << "  Rotation[1 2]= " << new_pose->R_wr_cl[5]
         << "  Rotation[2 0]= " << new_pose->R_wr_cl[6]
         << "  Rotation[2 1]= " << new_pose->R_wr_cl[7]
         << "  Rotation[2 2]= " << new_pose->R_wr_cl[8];

  queue_recv();
}

void PoseUdpService::service_thread() {
  io_service_.run();
}

void PoseUdpService::queue_recv() {
  socket_.async_receive(
      boost::asio::buffer(recv_buffer_),
      boost::bind(&PoseUdpService::read_handler, this, _1, _2));
}

void PoseUdpService::bind() {
  boost::system::error_code error;
  socket_.open(boost::asio::ip::udp::v4(), error);
  boost::asio::ip::udp::endpoint local(
      boost::asio::ip::address_v4::any(),
      port_);
  socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
  socket_.bind(local);
}

void PoseUdpService::start() {
  bind();
  queue_recv();
  // Start the thread that will run the associated I/O service.
  std::thread t(boost::bind(&PoseUdpService::service_thread, this));
  t.detach();
}

void PoseUdpService::triggerNextFrame() {
  pose_.swap(received_pose_);
}
