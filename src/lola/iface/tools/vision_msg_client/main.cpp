#include<sockets_common.hpp>

#include <sys/types.h>
#include <signal.h>

#include <algorithm>
#include <string>
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>

#include <tclap/CmdLine.h>
#include <iface_msg.hpp>
#include <iface_vision_msg.hpp>

using am2b_iface::VisionMessageHeader;
using am2b_iface::Message_Type;
using am2b_iface::VisionMessage;
using am2b_iface::ObstacleMessage;
using am2b_iface::ObstacleType;
using am2b_iface::SurfaceMessage;



/**
 * A tool for testing network communication between LEPP and other components
 *
 * This client will connect to a server at a given address and port,
 * and once connected will begin sending VisionMessages are regular intervals.
 *
 * This can be used to verify if data is being received correctly, or as a reference
 * when implementing other components which need to receive VisionMessage data.
 *
 *
**/

// maximum # of bytes to receive at once
#define BUFLEN 2048

struct ParsedParams
{
  unsigned int port = 0; // port to use
  std::string host_ip;   // server ip to connect to
  bool verbose = false;
};

bool parse_args(int argc, char* argv[], ParsedParams* params)
{
  try {
    TCLAP::CmdLine cmd("Vision Message Client", ' ', "0.4");

    TCLAP::ValueArg<unsigned int> portArg("p","port","Port to send data on",true,0,"unsigned int");
    TCLAP::ValueArg<std::string>  hostArg("n","host","Hostname to connect to",true,"localhost","string");

    cmd.add( portArg );
    cmd.add( hostArg );

    TCLAP::SwitchArg verboseSwitch("v","verbose","Verbose output", cmd, false);

    // Parse the argv array.
    cmd.parse( argc, argv );

    // Get the value parsed by each arg.
    params->port = portArg.getValue();
    params->host_ip = hostArg.getValue();
    params->verbose = verboseSwitch.getValue();
    } catch (TCLAP::ArgException &e)  // catch any exceptions
    {
       std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
       return false;
     }
  return true;
}

void failWithError(std::string s)
{
#ifdef _WIN32
  LPWSTR *errstr = NULL;
  FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                 NULL, WSAGetLastError(),
                 MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                 (LPWSTR)&s, 0, NULL);
  std::cerr << s << errstr << std::endl;
  LocalFree(errstr);
#else
  perror(s.c_str());
#endif
  exit(1);
}


// connects to hostname:port, and returns an open socket on success
int connect(unsigned int port, std::string hostname, bool verbose)
{
  struct addrinfo hints, *res;
  struct sockaddr_in server_addr;
  int s;

  // get server info for connection
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;
  int n = getaddrinfo(hostname.c_str(), std::to_string(port).c_str(), &hints, &res);
  if (n != 0)
    failWithError("Could not get host info!");

  std::cout << "Attempting to connect to: " << hostname << ":" << port << std::endl;

  // connect to server
  bool connection_success = false;
  for (auto rp = res; rp != NULL; rp = rp->ai_next) // check all addresses found by getaddrinfo
  {
    s = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);

#ifdef _WIN32
    if (s == INVALID_SOCKET)
#else
    if (s < 0)
#endif
      continue; // error creating socket

    // attempt to connect
    if (connect(s, rp->ai_addr, rp->ai_addrlen) == 0)
    {
      std::cout << "Connection success!" << std::endl;
      connection_success = true;
      break; // connection success!
    }
    else
    {
#ifdef _WIN32
        LogWSAErrorStr(WSAGetLastError());
#else
      perror("Could not connect");
#endif
    }

    socket_close(s); // if we failed, close connection and try the next one
  }

  if (!connection_success)
  {
    std::cout << "Connection failed!" << std::endl;
    exit(0);
  }

  std::cout << "Connected to server '" << hostname << "' on port: " << port << std::endl;
  freeaddrinfo(res);
  return s;
}

// sends data to the given socket, s
void transmit(int s, bool verbose)
{
#ifndef _WIN32
  // ignore SIGPIPE so we can print errors on write fails
  signal(SIGPIPE, SIG_IGN);
#endif

  // obstacles to send
  std::vector<double> sphere_coeffs  = {2.0, 2.0, 0.0,  0.0,  0.0,  0.0, 0.0, 0.0, 0.0};
  std::vector<double> capsule_coeffs = {1.0, 1.0, 1.0, -1.0, -1.0, -1.0, 0.0, 0.0, 0.0};
  std::vector<VisionMessage> messages = {
    VisionMessage(ObstacleMessage::SetMessage(ObstacleType::Sphere, 0, 1, 0.1, sphere_coeffs), 1),
    VisionMessage(ObstacleMessage::SetMessage(ObstacleType::Capsule, 1, 1, 0.15, capsule_coeffs), 1),
    VisionMessage(ObstacleMessage::ModifyMessage(ObstacleType::Sphere, 0, 1, 0.2, sphere_coeffs), 2),
    VisionMessage(ObstacleMessage::ModifyMessage(ObstacleType::Capsule, 1, 1, 0.05, capsule_coeffs), 2),    
    VisionMessage(ObstacleMessage::ModifyMessage(ObstacleType::Sphere, 0, 2, 0.2, sphere_coeffs), 3),
    VisionMessage(ObstacleMessage::ModifyMessage(ObstacleType::Capsule, 1, 2, 0.05, capsule_coeffs), 3),
    VisionMessage(ObstacleMessage::DeletePartMessage(0,2), 4),
    VisionMessage(ObstacleMessage::DeletePartMessage(1,2), 4),
    VisionMessage(ObstacleMessage::DeleteMessage(0), 5),
    VisionMessage(ObstacleMessage::DeleteMessage(1), 5)
  };

  // NOTE: as long as we're only sending obstacles we can use the same iface header for all of them...
  //       But if we update this to also send surfaces, we will need different headers due to different message sizes
  am2b_iface::MsgHeader iface_header = { am2b_iface::VISION_MESSAGE, sizeof(VisionMessageHeader) + sizeof(ObstacleMessage) };


  // loop over messages, sending one each iteration, and repeating once we reach the end of the list
  for (int i = 0; /* loop forever */; i = (i+1)%messages.size())
  {

    int sent = 0;
    VisionMessage msg = messages[i];
    char* buf = (char*)(&msg);

    std::cout << std::endl << "Sending iface header: 0x" << std::hex << iface_header.id << std::dec 
              << " length " << iface_header.len << std::endl;
    std::cout << std::endl << "Sending message: " << msg << std::endl;

    
    // unsigned char* big_buf = new unsigned char[sizeof(iface_header) + sizeof(VisionMessageHeader) + msg.header.len];
    // memcpy(big_buf, &iface_header, sizeof(iface_header));
    // memcpy(big_buf + sizeof(iface_header), &msg.header, sizeof(VisionMessageHeader));
    // memcpy(big_buf + sizeof(iface_header) + sizeof(VisionMessageHeader), msg.content, msg.header.len);

    // sent = write(s, big_buf, sizeof(iface_header) + sizeof(VisionMessageHeader) + msg.header.len);
    // if (sent <= 0)
    //   failWithError("Failed to send big message!");
    // if (verbose)
    //   {
    //     std::cout << "Sent " << sent << " bytes (everything)" << std::endl;
    //   }
    
    // send the iface header
    sent = socket_send(s, (char*)&iface_header, sizeof(am2b_iface::MsgHeader));
    if (sent <= 0)
      failWithError("Failed to send iface header!");

    if (verbose)
    {
      std::cout << "Sent " << sent << " bytes (am2b_iface::MsgHeader)" << std::endl;
    }
    
     // send the header
    sent = socket_send(s, buf, sizeof(VisionMessageHeader));
    if (sent <= 0)
      failWithError("Failed to send data!");

    if (verbose)
    {
      std::cout << "Sent " << sent << " bytes (message header)" << std::endl;
    }

     // send the message content
    sent = socket_send(s, msg.content, msg.header.len);
    if (sent <= 0)
      failWithError("Failed to send data!");

    if (verbose)
    {
      std::cout << "Sent " << sent << " bytes (message content)" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); /// TODO: This should be configurable from cmd line
  }
}

int main(int argc, char* argv[])
{
  ParsedParams params;
  if (!parse_args(argc, argv, &params))
  {
    return 0;
  }

#ifdef _WIN32 // must init WinSock before using sockets on Windows
  WSADATA wsaData = { 0 };
  int res = WSAStartup(MAKEWORD(2, 2), &wsaData);
  if (res != 0)
  {
      std::cerr << "Could not init Winsock!" << std::endl;
      LogWSAErrorStr(res);
      return -1;
  }
#endif

  int socket = connect(params.port, params.host_ip, params.verbose);
  transmit(socket, params.verbose);
  socket_close(socket);

#ifdef _WIN32
  if (WSACleanup() != 0)
      failWithError("Failed to shut down Winsock cleanly");
#endif

  return 0;
}
