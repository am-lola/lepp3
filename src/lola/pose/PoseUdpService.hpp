#ifndef LOLA_POSE_UDP_SERVICE_H__
#define LOLA_POSE_UDP_SERVICE_H__

#include "lola/PoseService.h"
#include <cstdint>
#include <memory>
#include <string>
#include <boost/asio.hpp>
#include <boost/array.hpp>

/**
 * A class that provides the ability to run a local service that listens to
 * LOLA pose messages on a particular UDP port. It provides an API for other
 * components to get the current pose information, without worrying about running
 * the networking communication infrastructure or threading.
 */
class PoseUdpService : public PoseService {
public:
  /**
   * Create a new `PoseService` that will listen on the given local (UDP) socket
   * for new pose messages coming from the robot. It does not need to know the
   * network address of the robot itself.
   */
  PoseUdpService(std::string const &host, uint16_t port)
      : host_(host),
        port_(port),
        socket_(io_service_),
        pose_counter_(0) {}

  virtual ~PoseUdpService();

  /**
   * Starts the `PoseService`.
   *
   * This spins up a thread behind the scenes in order to allow the messages to
   * be processed asynchronously.
   */
  void start() override;

  /**
   * Obtains the current pose information. Using this method is completely
   * thread safe.
   */
  HR_Pose_Red getCurrentPose() const override {
    // This does an atomic copy of the pointer (the refcount is atomically updated)
    // There can be no race condition since if the service needs to update the
    // pointer, it will do so atomically and the reader also obtains a copy of the
    // pointer atomically.
    std::shared_ptr<HR_Pose_Red> p = pose_;
    // Now we are safe to manipulate the object itself, since nothing else needs
    // to directly touch the instance itself and we have safely obtained a
    // reference to it.
    // We just return the object (but this involves a non-atomic copy, hence the
    // pointer dance).

    if (p) {
      return *p;
    } else {
      HR_Pose_Red pose = {0};
      return pose;
    }
  }

private:
  /**
   * Internal helper method. The callback that is passed to the async receive.
   */
  void read_handler(
      const boost::system::error_code &ec,
      std::size_t bytes_transferred);

  /**
   * Internal helper method. Represents the function that will be ran in the
   * service thread that is spawned once `start` is called.
   */
  void service_thread();

  /**
   * Binds the `socket_` to the local address represented by the parameters
   * given in the constructor.
   */
  void bind();

  /**
   * Queues a new asynchronous receive operation. After each handled receive a
   * new one needs to be queued if we wish to keep running the pose service.
   */
  void queue_recv();

  /**
   * The local host on which the service will listen.
   */
  std::string const host_;
  /**
   * The port number where the service will listen.
   */
  uint16_t const port_;

  /**
   * The IO service that will handle the async reads and callback invocation.
   */
  boost::asio::io_service io_service_;

  /**
   * The socket used for UDP communication.
   */
  boost::asio::ip::udp::socket socket_;
  /**
   * The buffer into which the socket will place its read data.
   */
  boost::array<char, sizeof(HR_Pose_Red)> recv_buffer_;

  /**
   * A pointer to the last known pose information.
   * Updated by the service on every newly received packet.
   */
  std::shared_ptr<HR_Pose_Red> pose_;

  /**
   * Current pose count
   */
  int pose_counter_;
};

#endif
