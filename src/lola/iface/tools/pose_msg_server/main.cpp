#include<sockets_common.hpp>

#ifndef _WIN32
#include <ifaddrs.h>
#else
#include <iphlpapi.h>
#endif

#include <sys/types.h>
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
 * This UDP server will broadcast POSE data on the given port.
 *
 *
**/

// maximum # of bytes to receive at once
#define BUFLEN 2048

unsigned long SockAddrToUint32(struct sockaddr* a)
{
  return ((a)&&(a->sa_family == AF_INET)) ? ntohl(((struct sockaddr_in *)a)->sin_addr.s_addr) : 0;
}

struct ParsedParams
{
  unsigned int port = 0; // port to listen on
  bool verbose = false;
};

bool parse_args(int argc, char* argv[], ParsedParams* params)
{
  try {
    TCLAP::CmdLine cmd("Pose Data Server", ' ', "0.4");

    TCLAP::ValueArg<unsigned int> portArg("p","port","Port to send data on",true,0,"unsigned int");

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
  std::perror(s.c_str());
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
  if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
    failWithError("creating socket failed!");

  if (setsockopt(s, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast))!= 0)
    failWithError("Setting broadcast flag on socket failed!");

  if (setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &reuseport, sizeof(reuseport))!= 0)
    failWithError("Setting Reuse Addr flag on socket failed!");

  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(port);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  // if (bind(s, (sockaddr*)&si_me, sizeof(si_me))==-1)
  //   failWithError("binding socket failed!");

  return s;
}

void send_pose_data(socklen_t s, unsigned int port, bool verbose)
{
  struct sockaddr_in si_other;
  struct ifaddrs* ifap;

#ifdef _WIN32
  PIP_ADAPTER_ADDRESSES pAddresses   = NULL;
  ULONG outBufLen = 0;
  
  auto res = GetAdaptersAddresses(AF_INET, GAA_FLAG_INCLUDE_PREFIX, NULL, pAddresses, &outBufLen);
  if (res == ERROR_BUFFER_OVERFLOW)
  {
      free(pAddresses);
      pAddresses = (IP_ADAPTER_ADDRESSES*)malloc(outBufLen);
      res = GetAdaptersAddresses(AF_INET, GAA_FLAG_INCLUDE_PREFIX, NULL, pAddresses, &outBufLen);
  }

  if (res != NO_ERROR)
  {
      std::cout << "ERROR: Could not enumerate network interfaces. Will only broadcast on the default interface." << std::endl;
      pAddresses = NULL;
  }
#else
  if (getifaddrs(&ifap) != 0)
  {
      std::cout << "ERROR: Could not enumerate network interfaces. Will only broadcast on the default interface." << std::endl;
      ifap = NULL;
  }
#endif

  

  si_other.sin_family = AF_INET;
  si_other.sin_port = htons(port);
  si_other.sin_addr.s_addr = htonl(INADDR_BROADCAST);

  // fake neutral pose (?)
  HR_Pose_Red pose;
  pose.version = 1;
  pose.tick_counter = 1;
  pose.stance = 0;
  pose.stamp = 1;
  pose.t_wr_cl[0] = 0;
  pose.t_wr_cl[1] = 0;
  pose.t_wr_cl[2] = 1.78;
  pose.R_wr_cl[0] = 0;
  pose.R_wr_cl[1] = -1;
  pose.R_wr_cl[2] = 0;
  pose.R_wr_cl[3] = -0.836;
  pose.R_wr_cl[4] = -0.032;
  pose.R_wr_cl[5] = -0.548;
  pose.R_wr_cl[6] = 0.548;
  pose.R_wr_cl[7] = 0.036;
  pose.R_wr_cl[8] = -0.837;
  pose.t_stance_odo[0] = 0;
  pose.t_stance_odo[1] = 0;
  pose.t_stance_odo[2] = 0;
  pose.phi_z_odo = 0;


  while (1)
  {
    ssize_t sent = 0;

#ifdef _WIN32
    if (pAddresses == NULL)
#else
    if (ifap == NULL)
#endif
    {
        if ((sent = sendto(s, (char*)&pose, sizeof(pose), 0, (struct sockaddr*)&si_other, sizeof(si_other))) < 0)
          failWithError("Failed to send pose data!");
    }
    else
    {
#ifdef _WIN32
        PIP_ADAPTER_ADDRESSES currAddress = pAddresses;
        while (currAddress != NULL)
        {
            std::wcout << "sending on interface: " << currAddress->Description << std::endl; // AdapterName << std::endl;
            si_other.sin_addr = ((sockaddr_in*)(currAddress->FirstUnicastAddress->Address.lpSockaddr))->sin_addr;
            if ((sent = sendto(s, (char*)&pose, sizeof(pose), 0, (struct sockaddr*)&si_other, sizeof(si_other))) < 0)
                failWithError("Failed to send pose data!");

            currAddress = currAddress->Next;
        }
#else
        struct ifaddrs* iface = ifap;
        while (iface)
        {
          if (SockAddrToUint32(iface->ifa_addr) > 0)
          {
            std::cout << "sending on interface: " << iface->ifa_name << std::endl;
            si_other.sin_addr = ((sockaddr_in*)(iface->ifa_dstaddr))->sin_addr;
            if ((sent = sendto(s, (char*)&pose, sizeof(pose), 0, (struct sockaddr*)&si_other, sizeof(si_other))) < 0)
              failWithError("Failed to send pose data!");
          }
          iface = iface->ifa_next;
        }
#endif
    }
    std::cout << "Sent one HR_Pose_Red (" << sent << " bytes)" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

  std::cout << "Socket opened, broadcasting on port " << params.port << "..." << std::endl;

  send_pose_data(sock, params.port, params.verbose);

  socket_close(sock);

#ifdef _WIN32
  if (WSACleanup() != 0)
      failWithError("Failed to shut down Winsock cleanly");
#endif

  return 0;
}
