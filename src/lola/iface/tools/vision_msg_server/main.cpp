#include<sockets_common.hpp>

#include <sys/types.h>

#include <algorithm>
#include <string>
#include <vector>
#include <iostream>

#include <tclap/CmdLine.h>
#include <iface_msg.hpp>
#include <iface_vision_msg.hpp>

using am2b_iface::VisionMessageHeader;
using am2b_iface::Message_Type;
using am2b_iface::VisionMessage;
using am2b_iface::ObstacleMessage;
using am2b_iface::SurfaceMessage;
using am2b_iface::PointCloudMessage;
using am2b_iface::RGBMessage;


/**
 * A tool for testing network communication between LEPP and other components
 *
 * This server will listen on a given port for a TCP connection from LEPP,
 * and once connected will receive and decode VisionMessages containing
 * obstacles, surfaces, etc.
 *
 * This can be used to verify if data is being sent correctly, or as a reference
 * when implementing other components which need to receive VisionMessage data.
 *
 *
**/

// maximum # of bytes to receive at once
#define BUFLEN 2048

struct ParsedParams
{
  unsigned int port = 0; // port to listen on
  bool verbose = false;
};


bool parse_args(int argc, char* argv[], ParsedParams* params)
{
  try {
    TCLAP::CmdLine cmd("Vision Message Server", ' ', "0.4");

    TCLAP::ValueArg<unsigned int> portArg("p","port","Port to listen on",true,0,"unsigned int");
    cmd.add( portArg );
    TCLAP::SwitchArg verboseSwitch("v","verbose","Verbose output", cmd, false);

    // Parse the argv array.
    cmd.parse( argc, argv );
    // Get the value parsed by each arg.
    params->port = portArg.getValue();
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

void readDataFrom(int socket_remote, const sockaddr_in& si_other, bool verbose)
{
  std::vector<char> buf;
  buf.resize(BUFLEN); // init buffer to be at least BUFLEN; we'll expand it later if need be

  while (1)
  {
    ssize_t const iface_headerSize = sizeof(am2b_iface::MsgHeader);
    ssize_t const vision_headerSize = sizeof(VisionMessageHeader);
    ssize_t total_received = 0;
    ssize_t total_expected = iface_headerSize;

    if (verbose)
    {
      std::cout << "Waiting for am2b_iface::MsgHeader (" << iface_headerSize << " bytes)..." << std::endl;
    }

    while (total_received < iface_headerSize)
    {
      int recvd = 0;
      recvd = socket_recv(socket_remote, &buf[total_received], total_expected - total_received);

      if (recvd == 0)
        return;
      if (recvd == -1)
        failWithError("read() failed!");

      total_received += recvd;

      if (verbose)
      {
        std::printf("(iface header) Received %zu / %zu total bytes from %s:%d\n", total_received, total_expected, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
      }
    }

    am2b_iface::MsgHeader* iface_header = (am2b_iface::MsgHeader*)buf.data();

    if (verbose)
    {
      std::cout << "Waiting for rest of message (" << iface_header->len << " bytes)..." << std::endl;
    }

    total_expected += iface_header->len;
    // make sure we have enough space to hold the rest of the message
    if (buf.size() < total_expected)
    {
      if (verbose)
      {
        std::cout << "Resizing receive buffer to " << total_expected << " bytes" << std::endl;
      }
      buf.resize(total_expected);
      iface_header = (am2b_iface::MsgHeader*)buf.data(); // update pointer to new allocation
    }

    while (total_received < total_expected)
    {
      int recvd = 0;
      recvd = socket_recv(socket_remote, &buf[total_received], total_expected-total_received);

      if (recvd == 0) // connection died
        return;
      if (recvd == -1) // failed to read from socket
        failWithError("read() failed!");

      total_received += recvd;

      if (verbose)
      {
        std::printf("(msg) Received %zu / %zu total bytes from %s:%d\n", total_received, total_expected, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
      }
    }

    if (iface_header->id != am2b_iface::VISION_MESSAGE)
    {
      if (verbose)
      {
        std::cout << "Received non-vision message of type: 0x" << std::hex << iface_header->id << std::dec << ". Skipping it..." << std::endl;
      }
      continue;
    }

    VisionMessageHeader* header = (VisionMessageHeader*)(buf.data() + sizeof(am2b_iface::MsgHeader));
    std::cout << "Received VisionMessageHeader: " << *header << std::endl;

    switch (header->type)
    {
      case Message_Type::Obstacle:
      {
        ObstacleMessage* message = (ObstacleMessage*)(buf.data() + sizeof(VisionMessageHeader) + sizeof(am2b_iface::MsgHeader));
        std::cout << "Received obstacle: " << std::endl;
        std::cout << *message << std::endl;;
        break;
      }
      case Message_Type::Surface:
      {
        SurfaceMessage* message = (SurfaceMessage*)(buf.data() + sizeof(VisionMessageHeader) + sizeof(am2b_iface::MsgHeader));
        std::cout << "Received Surface:" << std::endl;
        std::cout << "\tid: " << message->id << std::endl;
        std::cout << "\taction: 0x" << std::hex << message->action << std::dec << std::endl;
        std::cout << "\tnormal: [" << message->normal[0] << ", " << message->normal[1] << ", " << message->normal[2] << "]" << std::endl;
        std::cout << "\tVertices:" << std::endl;
        for (int i = 0; i < 8; i++)
        {
          std::cout << "\t\t[";
          std::cout << message->vertices[i*3] << ", " << message->vertices[i*3 + 1] << ", " << message->vertices[i*3 + 2];
          std::cout << "]" << std::endl;
        }
        break;
      }
      case Message_Type::PointCloud:
      {
        PointCloudMessage* message = (PointCloudMessage*)(buf.data() + sizeof(VisionMessageHeader) + sizeof(am2b_iface::MsgHeader));
        std::cout << "Received PointCloud:" << std::endl;
        std::cout << "\tNumber of points: " << message->count << std::endl;
        std::cout << "\tSize of points:   " << message->format << " bytes" << std::endl;
        break;
      }
      case Message_Type::RGB_Image:
      {
        RGBMessage* message = (RGBMessage*)(buf.data() + sizeof(VisionMessageHeader) + sizeof(am2b_iface::MsgHeader));
        std::cout << "Received RGB Image:" << std::endl;
        std::cout << "\tWidth:  " << message->width  << std::endl;
        std::cout << "\tHeight: " << message->height << std::endl;
        break;
      }
      default:
      {
        std::cout << "UNKNOWN VisionMessage type: " << header->type << "!!" << std::endl;
      }
    }

  }
}

void listen(unsigned int port, bool verbose)
{
  struct sockaddr_in si_me, si_other;
  int s, s_other;
  socklen_t slen=sizeof(si_other);

  // create & bind socket
  s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

#ifdef _WIN32
  if ( s == INVALID_SOCKET)
#else
  if ( s == -1 )
#endif
    failWithError("creating socket failed!");

  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(port);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (sockaddr*)&si_me, sizeof(si_me))==-1)
    failWithError("bind failed!");

  if (listen(s, 5) != 0)
    failWithError("listen failed!");

  while(1)
  {
    std::cout << "Listening for connection on port " << port << "..." << std::endl;

    s_other = accept(s, (struct sockaddr* ) &si_other, &slen);
    if (s_other < 0)
      failWithError("accept failed!");

    std::cout << std::endl << "-------------------------------------------------" << std::endl;
    std::cout << "NEW Connection to " << inet_ntoa(si_other.sin_addr) << ":" << htons(si_other.sin_port) << std::endl;
    std::cout << "-------------------------------------------------" << std::endl << std::endl;


    // receive data from new connection
    readDataFrom(s_other, si_other, verbose);

    std::cout << "Connection to client terminated!" << std::endl;
    std::cout << "-------------------------------------" << std::endl << std::endl;;
    socket_close(s_other);
  }

  socket_close(s);
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

  listen(params.port, params.verbose);

#ifdef _WIN32
  if (WSACleanup() != 0)
      failWithError("Failed to shut down Winsock cleanly");
#endif

  return 0;
}
