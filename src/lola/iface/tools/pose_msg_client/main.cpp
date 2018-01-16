#include<sockets_common.hpp>

#include <sys/types.h>

#include <string.h>
#include <algorithm>
#include <string>
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>

#include <tclap/CmdLine.h>

// contains defintions for HR_Pose, HR_Pose_Red, etc
#include <iface_vis.h>


/**
 * A tool for testing network communication between LEPP and other components
 *
 * This UDP client will receive POSE data on the given port.
 *
 *
**/

// maximum # of bytes to receive at once
#define BUFLEN 512

struct ParsedParams
{
  unsigned int port = 0; // port to listen on
  bool verbose = false;
};

bool parse_args(int argc, char* argv[], ParsedParams* params)
{
  try {
    TCLAP::CmdLine cmd("Pose Data Client", ' ', "0.4");

    TCLAP::ValueArg<unsigned int> portArg("p","port","Port to listen on for data",true,0,"unsigned int");

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
  std::perror(s.c_str());
#endif
  exit(1);
}

socklen_t init_socket(unsigned int port, bool verbose)
{
  struct sockaddr_in si_me;
  socklen_t s;
#ifdef _WIN32
  char broadcast = 1;
  char reuseport = 1;
#else
  int broadcast = 1;
  int reuseport = 1;
#endif

  // create & bind socket
  s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#ifdef _WIN32
  if ( s == INVALID_SOCKET )
#else
  if ( s ==-1 )
#endif
    failWithError("creating socket failed!");

  if (setsockopt(s, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast))!= 0)
    failWithError("Setting broadcast flag on socket failed!");

  if (setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &reuseport, sizeof(reuseport))!= 0)
    failWithError("Setting Reuse Addr flag on socket failed!");

  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(port);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (sockaddr*)&si_me, sizeof(si_me))==-1)
    failWithError("binding socket failed!");

  return s;
}

void printvec(float* vec, unsigned int len, std::ostream& out)
{
  out << "[";
  for (unsigned int i = 0; i < len; i++)
  {
    out << vec[i];

    if (i < len-1)
      out << ", ";
  }
  out << "]";
}

void printmat(float* mat, unsigned int width, unsigned int height, std::string line_prefix, std::ostream& out)
{

  for (unsigned int i = 0; i < width; i++)
  {
    out << line_prefix << "[";
    for (unsigned int j = 0; j < height; j++)
    {
      out << mat[width*i+j];
      if (j < height-1)
        out << ", ";
    }
    out << "]";
    out << std::endl;
  }
}

void receive_pose_data(socklen_t s, bool verbose)
{
  char buf[BUFLEN];
  sockaddr_in si_other;
  socklen_t slen = sizeof(sockaddr);
  while(1)
  {
    // clear buffer
    memset(buf, 0, BUFLEN-1);

    // wait for message
    int nrecvd = recvfrom(s, buf, BUFLEN-1,0, (sockaddr*)&si_other, &slen);

    if (verbose)
      std::cout << "Received " << nrecvd << " bytes from: " << inet_ntoa(si_other.sin_addr) << std::endl;

    // print received pose data
    HR_Pose_Red* new_pose = (HR_Pose_Red*)buf;
    std::cout << "New Pose:" << std::endl;
    std::cout << "\tVersion: " << new_pose->version << std::endl;
    std::cout << "\tTick Counter: " << new_pose->tick_counter << std::endl;
    std::cout << "\tStance: " << (unsigned int)(new_pose->stance) << std::endl;
    std::cout << "\tStamp:  " << new_pose->stamp << std::endl;
    std::cout << "\tt_wr_cl:" << std::endl;
    std::cout << "\t\t"; printvec(new_pose->t_wr_cl, 3, std::cout); std::cout << std::endl;
    std::cout << "\tR_wr_cl:" << std::endl;
    printmat(new_pose->R_wr_cl, 3, 3, "\t\t", std::cout);
    std::cout << "\tt_wr_ub:" << std::endl;
    std::cout << "\t\t"; printvec(new_pose->t_wr_ub, 3, std::cout); std::cout << std::endl;
    std::cout << "\tR_wr_ub:" << std::endl;
    printmat(new_pose->R_wr_ub, 3, 3, "\t\t", std::cout);
    std::cout << "\tt_stance_odo: " << std::endl;;
    std::cout << "\t\t"; printvec(new_pose->t_stance_odo, 3, std::cout); std::cout << std::endl;
    std::cout << "\tphi_z_odo: " << new_pose->phi_z_odo << std::endl;
    std::cout << "------------------------------------------" << std::endl;
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

  socklen_t sock = init_socket(params.port, params.verbose);

  std::cout << "Socket opened, receiving on port " << params.port << "..." << std::endl;

  receive_pose_data(sock, params.verbose);

  socket_close(sock);

#ifdef _WIN32
  if (WSACleanup() != 0)
      failWithError("Failed to shut down Winsock cleanly");
#endif

  return 0;
}
