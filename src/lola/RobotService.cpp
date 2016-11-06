#include "RobotService.h"
#include <boost/thread.hpp>

#include "deps/easylogging++.h"
#include <iface_msg.hpp>

namespace {
  /**
   * Callback invoked when the async connect operation completes.
   */
  void connect_handler(boost::system::error_code const& error) {
    if (!error) {
      LINFO << "AsyncRobotService: Connected to remote host.";
    } else {
      LERROR << "AsyncRobotService:  Failed to connect to the remote host.";
    }
  }

  /**
   * A simple function that is used to spin up the io service event loop in a
   * dedicated thread.
   * Makes sure that the loop does not end when the service does not have any
   * work to do (temporarily).
   */
  void service_thread(boost::asio::io_service* io_service) {
    // Prevent the IO service from running out of work (and exitting when no
    // async operations are queued).
    boost::asio::io_service::work work(*io_service);
    io_service->run();
    LINFO << "AsyncRobotService: Exiting service thread...";
  }

  /**
   * A callback for write operations.
   */
  void write_handler(boost::system::error_code const& error,
               std::size_t sent) {
    if (!error) {
      LINFO << "AsyncRobotService: Send complete. "
            << "Sent " << sent << " bytes.";
    } else {
      LERROR << "AsyncRobotService: Error sending message.";
    }
  }
}

void AsyncRobotService::start() {
  // Start it up...
  boost::asio::ip::tcp::endpoint endpoint(
    boost::asio::ip::address::from_string(remote_), port_);
  LINFO << "AsyncRobotService (" << remoteName_ << "): Initiating a connection asynchronously...";
  socket_.async_connect(endpoint, connect_handler);
  // Start the service thread in the background...
  boost::thread(boost::bind(service_thread, &io_service_));
}

void AsyncRobotService::inner_send(VisionMessage const& next_message) {
  am2b_iface::MsgHeader msg_header = { am2b_iface::VISION_MESSAGE, (uint32_t)sizeof(VisionMessageHeader) + next_message.header.len };
  LINFO << "AsyncRobotService (" << remoteName_ << "): Sending a queued message: "
        << "msg == " << next_message;
  // Synchronously send the message, i.e. block until the send is complete.
  try {
    sendBytes(reinterpret_cast<char const*>(&msg_header), sizeof(msg_header));
    sendBytes(reinterpret_cast<char const*>(&next_message), sizeof(VisionMessageHeader));
    sendBytes(next_message.content, next_message.header.len);
  } catch (...) {
    LERROR << "AsyncRobotService (" << remoteName_ << "): Error sending message.";
  }
  // After each sent message, we want to wait a pre-defined amount of time
  // before sending the next one.
  // This is because we do not want to overwhelm the robot with a large
  // number of messages all sent in the same time.
  boost::this_thread::sleep(message_timeout_);
}

void AsyncRobotService::sendBytes(char const* data, size_t length) {
  size_t sent_bytes = 0;

  do {
    sent_bytes += socket_.send(boost::asio::buffer(&data[sent_bytes], length - sent_bytes));
  } while (sent_bytes < length);
}

void AsyncRobotService::sendMessage(VisionMessage const& msg) {
  // Just queue another message to be sent by the io_service thread.
  // Since `inner_send` makes sure to wait after it's finished sending
  // each message (i.e. the io_service thread is blocked), the queued
  // messages will all be sent with a preset time delay between subsequent
  // messages.
  io_service_.post(boost::bind(&AsyncRobotService::inner_send, this, msg));
}
