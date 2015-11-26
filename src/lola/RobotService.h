#ifndef LOLA_ROBOT_SERVICE_H__
#define LOLA_ROBOT_SERVICE_H__

#include <boost/asio.hpp>
#include <cstring>
#include <iostream>

// The macro creates an ID for a Robot message.
// The macro is taken from the LOLA source base.
// TODO Once C++11 can be used, make this a `constexpr` function, instead of a macro.
#define __MSG_ID_DEF_GLOBAL(dom,sig)  (0x80000000 | ((dom&0xFF)<<0x10) | (sig&0xFFFF))

/**
 * A struct representing the raw vision message format that is sent to the
 * robot.
 */
struct VisionMessage {
  uint32_t id;
  uint32_t len;
  float params[15];

  VisionMessage() : len(sizeof params) {}

  // IDs for particular vision operations.
  static uint32_t const SET_SSV =  __MSG_ID_DEF_GLOBAL(0x4, 0x203);
  static uint32_t const MODIFY_SSV =  __MSG_ID_DEF_GLOBAL(0x4, 0x206);
  static uint32_t const REMOVE_SSV =  __MSG_ID_DEF_GLOBAL(0x4, 0x207);

  // Flags (passed in parameter at the index 4 in delete messages) indicating
  // whether the entire object model should be removed or only a part.
  static float const DEL_WHOLE_SEGMENT_FLAG = 0;
  static float const DEL_ONLY_PART_FLAG = 1;

  // Static factory functions. Facilitate creating the messages without worrying
  // about the internal format.
  /**
   * Creates a `VisionMessage` that says that an object with the given ID
   * should be removed. The entire model is removed, including all of its child
   * parts.
   */
  static VisionMessage DeleteMessage(int model_id);
  /**
   * Creates a `VisionMessage` that says that a particular part of a larger model
   * should be deleted.
   */
  static VisionMessage DeletePartMessage(int model_id, int part_id);
  /**
   * Creates a `VisionMessage` that says that a new object with the given
   * parameters should be created.
   */
  static VisionMessage SetMessage(
      int type_id, int model_id, int part_id, double radius, std::vector<double> const& coefs);
  /**
   * Creates a `VisionMessage` that says that an existing object with the given
   * ID should be modified according to the given parameters.
   */
  static VisionMessage ModifyMessage(
      int type_id, int model_id, int part_id, double radius, std::vector<double> const& coefs);
};

std::ostream& operator<<(std::ostream& out, VisionMessage const& msg);

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
   * The host name of the robot.
   */
  std::string const remote_;
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
