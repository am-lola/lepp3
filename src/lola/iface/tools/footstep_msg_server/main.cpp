#include <sockets_common.hpp>

#include <sys/types.h>
#include <signal.h>
#include <string.h>

#include <algorithm>
#include <string>
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>

#include <tclap/CmdLine.h>
#include <iface_msg.hpp>
#include <iface_stepseq.hpp>
#include <iface_sig_wpatt.hpp>

/**
 * A tool for testing network communication between QNX and other components
 *
 * This client will connect to a server at a given address and port,
 * and once connected will begin sending footsteps are regular intervals.
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
  bool verbose = false;
};

bool parse_args(int argc, char* argv[], ParsedParams* params)
{
  try {
    TCLAP::CmdLine cmd("Footstep Message Client", ' ', "0.4");

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

void failWithError(std::wstring s)
{
#ifdef _WIN32
  LPWSTR errstr = NULL;
  int err = WSAGetLastError();
  FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
      NULL, err,
      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
      (LPWSTR)&errstr, 0, NULL);
  std::wcerr << s << std::endl << "\tError " << err << " : " << std::wstring(errstr) << std::endl;
  LocalFree(errstr);
#else
  std::wcerr << s << std::endl << "\tError ";
  perror(NULL);
#endif
  exit(1);
}


// sends data to the given socket, s
void transmit(int s, bool verbose)
{
#ifndef _WIN32
  // ignore SIGPIPE so we can print errors on write fails
  signal(SIGPIPE, SIG_IGN);
#endif

  am2b_iface::MsgHeader iface_header = { am2b_iface::STEPSEQ_AR_VIZUALIZATION, sizeof(am2b_iface::struct_data_stepseq_ssv_log) };
  std::vector<am2b_iface::struct_data_stepseq_ssv_log> footsteps;
  for (size_t i = 0; i < 8; i++)
  {
    am2b_iface::struct_data_stepseq_ssv_log step;
    step.stamp_gen = 1000;
    step.start_x = i * 0.1;
    step.start_y = (i % 2 == 0) ? -0.1 : 0.1;
    step.start_z = 0;
    step.stance  = (i%2 == 0) ? 1 : 0;
    step.dz_clear = 0.04;
    step.dz_step = 0.04;
    footsteps.push_back(step);
  }


  // loop over messages, sending one each iteration, and repeating once we reach the end of the list
  int stamp_i = 1;
  for (int i = 0; /* loop forever */ ; i = (i+1)%footsteps.size())
  {
    if (i % footsteps.size() == 0)
        stamp_i++;

    int sent = 0;
    am2b_iface::struct_data_stepseq_ssv_log msg = footsteps[i];
    msg.stamp_gen += stamp_i;
    char* buf = (char*)(&msg);

    std::cout << "Sending header: { 0x" << std::hex << iface_header.id << std::dec << ", " << iface_header.len << " }" << std::endl;
     // send the header
    sent = socket_send(s, (char*)&iface_header, sizeof(am2b_iface::MsgHeader));
    if (sent <= 0)
      failWithError(L"Failed to send data!");

    if (verbose)
    {
      std::cout << "Sent " << sent << " bytes (am2b_iface::MsgHeader)" << std::endl;
    }
    
    std::cout << std::endl << "Sending message: " << i << std::endl;
    std::cout << "\tstamp_gen: " << msg.stamp_gen << std::endl;
    std::cout << "\t   stance: " << msg.stance << std::endl;
    std::cout << "\t  start_x: " << msg.start_x << std::endl;
    std::cout << "\t  start_y: " << msg.start_y << std::endl;
    std::cout << "\t  start_z: " << msg.start_z << std::endl;
    sent = socket_send(s, buf, sizeof(am2b_iface::struct_data_stepseq_ssv_log));
    if (sent <= 0)
      failWithError(L"Failed to send data!");

    if (verbose)
    {
      std::cout << "Sent " << sent << " bytes (message data)" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::seconds(2)); /// TODO: This should be configurable from cmd line
  }
}

void listen(unsigned int port, bool verbose)
{
    struct sockaddr_in si_me, si_other;
    int s, s_other;
    socklen_t slen = sizeof(si_other);

    // create & bind socket
    s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

#ifdef _WIN32
    if (s == INVALID_SOCKET)
#else
    if (s == -1)
#endif
        failWithError(L"creating socket failed!");

    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(port);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s, (sockaddr*)&si_me, sizeof(si_me)) == -1)
        failWithError(L"bind failed!");

    if (listen(s, 5) != 0)
        failWithError(L"listen failed!");

    while (1)
    {
        std::cout << "Listening for connection on port " << port << "..." << std::endl;

        s_other = accept(s, (struct sockaddr*) &si_other, &slen);
        if (s_other < 0)
            failWithError(L"accept failed!");

        std::cout << std::endl << "-------------------------------------------------" << std::endl;
        std::cout << "NEW Connection to " << inet_ntoa(si_other.sin_addr) << ":" << htons(si_other.sin_port) << std::endl;
        std::cout << "-------------------------------------------------" << std::endl << std::endl;


        transmit(s_other, verbose);

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
  /*int socket = connect(params.port, params.host_ip, params.verbose);
  transmit(socket, params.verbose);
  socket_close(socket);
*/
#ifdef _WIN32
  if (WSACleanup() != 0)
    failWithError(L"Failed to shut down Winsock cleanly");
#endif

  return 0;
}
