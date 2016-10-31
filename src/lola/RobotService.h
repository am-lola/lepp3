#ifndef LOLA_ROBOT_SERVICE_H__
#define LOLA_ROBOT_SERVICE_H__

#include <boost/asio.hpp>
#include <cstring>
#include <iostream>
#include "iface_vision_msg.hpp"

using am2b_iface::VisionMessage;
using am2b_iface::VisionMessageHeader;

/**
 * An interface that needs to be implemented by concrete classes that can
 * send vision messages to the robot.
 */
class RobotService {
public:
  /**
   * Send the given message to the robot.
   *
   * It is up to particular concrete `RobotService` implementations to decide
   * how the method goes about performing this operation and whether it blocks
   * or not.
   */
  virtual void sendMessage(VisionMessage const& msg) = 0;
};

/**
 * A class that implements a service which can send vision-related notifications
 * to the robot.
 *
 * It allows clients to asychronously send vision messages to the robot.
 */
class AsyncRobotService : public RobotService {
public:
  /**
   * Creates a new `AsyncRobotService` instance that will try to send messages
   * to a robot on the given remote address (host name, port combination).
   *
   * No delay between subsequent messages is set.
   */
  AsyncRobotService(std::string const& remote, int port)
      : remote_(remote), port_(port), socket_(io_service_),
        message_timeout_(0) {}

  /**
   * Creates a new `AsyncRobotService` instance that will try to send messages
   * to a robot on the given remote address (host name, port combination).
   *
   * The delay between each subsequent sent message is set by the `delay`
   * parameter.
   */
  AsyncRobotService(std::string const& remote, int port, int delay)
      : remote_(remote), port_(port), socket_(io_service_),
        message_timeout_(delay) {}
  AsyncRobotService(std::string const& remote, std::string const& remoteName, int port, int delay)
      : remote_(remote), remoteName_(remoteName), port_(port), socket_(io_service_),
        message_timeout_(delay) {}
  /**
   * Starts up the service, initiating a connection to the robot.
   *
   * Behind the scenes, this spins up a thread that will handle the IO for this
   * service.
   */
  void start();
  /**
   * Asynchronously sends a message to the robot.
   *
   * The call never blocks.
   */
  void sendMessage(VisionMessage const& msg);
private:
  /**
   * The host name to send data to.
   */
  std::string const remote_;
  /**
   * Friendly name of remote host for logging/debugging
   */
  std::string const remoteName_;
  /**
   * The port on which the robot is expecting vision messages.
   */
  int const port_;
  /**
   * The io service that handles the async operations of the communication.
   */
  boost::asio::io_service io_service_;
  /**
   * The socket that is connected to the remote robot endpoint.
   */
  boost::asio::ip::tcp::socket socket_;

  /**
   * A number of milliseconds that the service waits between subsequent
   * messages that it sends to the robot.
   */
  boost::posix_time::milliseconds message_timeout_;

  /**
   * A helper function that performs the send of the message and then waits
   * the predefined amount of time before returning. This way, the wait blocks
   * the (io_service) thread and causes the next message that should be sent
   * to be delayed the necessary amount of time.
   */
  void inner_send(VisionMessage const& msg);
};

#endif
