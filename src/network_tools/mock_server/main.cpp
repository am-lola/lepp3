#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include <getopt.h>
#include <algorithm>
#include <string>
#include <vector>
#include <iostream>

#include <iface_vision_msg.hpp>

using am2b::iface::VisionMessageHeader;
using am2b::iface::Message_Type;
using am2b::iface::VisionMessage;
using am2b::iface::ObstacleMessage;
using am2b::iface::SurfaceMessage;



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
  int c;

  if (argc < 2)
  {
    std::cout << "ERROR: You must provide at least a port!" << std::endl;
    return false;
  }

  while (1)
  {
    static struct option long_options[] =
    {
      {"port", required_argument, 0, 'p'},
      {"verbose", no_argument,    0, 'v'},
      {"help",    no_argument,    0, 'h'},
      {0}
    };

    int option_index = 0;
    c = getopt_long(argc, argv, "p:vh",
                    long_options, &option_index);

    /* detect end of options */
    if (c == -1)
      break;

    switch (c)
    {
      case 'p':
          params->port = std::stoi(optarg);
          break;

      case 'v':
          params->verbose = true;
          break;

      case 'h':
          return false;
          break;

      default:
          abort ();
    }
  }

  if (optind < argc)
  {
    std::cout << "Unrecognized arguments: ";
    while (optind < argc)
    {
      std::cout << argv[optind++] << ", ";
    }
      std::cout << std::endl;
    return false;
  }
  return true;
}

void printHelp(std::string name)
{
  std::cout << name << ": Test Application for receiving VisionMessage data." << std::endl << std::endl;
  std::cout << "Usage:" << std::endl;
  std::cout << "\t" << name << " --port Port-Number [--verbose] [--help]" << std::endl;
  std::cout << "\t" << name << " -p Port-Number [-v] [-h]" << std::endl;
  std::cout << "\t" << std::endl;
  std::cout << "\t" << "    port: TCP Port to listen on" << std::endl;
  std::cout << "\t" << "    help: Display this message" << std::endl;
  std::cout << std::endl;
}

void failWithError(std::string s)
{
  perror(s.c_str());
  exit(1);
}

void listen(unsigned int port, bool verbose)
{
  struct sockaddr_in si_me, si_other;
  int s, s_other;
  socklen_t slen=sizeof(si_other);

  // create & bind socket
  if ((s=socket(AF_INET, SOCK_STREAM, 0))==-1)
    failWithError("creating socket failed!");

  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(port);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (sockaddr*)&si_me, sizeof(si_me))==-1)
    failWithError("bind failed!");

  if (listen(s, 5) != 0)
    failWithError("listen failed!");

  std::cout << "Socket opened, listening for data on port " << port << "..." << std::endl;
  
  s_other = accept(s, (struct sockaddr* ) &si_other, &slen);
  if (s_other < 0)
    failWithError("accept failed!");


  unsigned char buf[BUFLEN];
  while (1)
  {
    std::fill(buf, buf+BUFLEN, 0);
    ssize_t headerSize = sizeof(VisionMessageHeader);
    ssize_t total_received = 0;

    if (verbose)
    {
      std::cout << "Waiting for VisionMessageHeader (" << headerSize << " bytes)..." << std::endl;
    }

    while (total_received < headerSize)
    {
      int recvd = 0;
      recvd = read(s_other, &buf[total_received], BUFLEN-total_received);

      if (recvd == -1)
        failWithError("read() failed!");

      total_received += recvd;

      if (verbose)
      {
        printf("(header) Received %zu total bytes from %s:%d\n", total_received, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
      }
    }

    VisionMessageHeader* header = (VisionMessageHeader*)buf;

    std::cout << "Received VisionMessageHeader:" << std::endl;
    std::cout << "\t type:" << header->type << ", len:" << header->len << std::endl;

    if (verbose)
    {
      std::cout << "Waiting to receive a total of " << header->len + sizeof(VisionMessageHeader) << " bytes..." << std::endl;
    }

    while (total_received < header->len + sizeof(VisionMessageHeader))
    {
      int recvd = 0;
      recvd = read(s_other, &buf[total_received], BUFLEN-total_received);
      if (recvd == -1)
        failWithError("read() failed!");

      total_received += recvd;
      if (verbose)
      {
        printf("(message) Received %zu total bytes from %s:%d\n", total_received, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
      }
    }

    switch (header->type)
    {
      case Message_Type::Obstacle:
      {
        ObstacleMessage* message = (ObstacleMessage*)(buf+sizeof(VisionMessageHeader));
        std::cout << "Received obstacle:" << std::endl;
        std::cout << "\t Type: " << message->type << " ID: " << message->model_id << "|" << message->part_id << std::endl;
        std::cout << "\t Action: 0x" << std::hex << message->action << std::dec << ", radius: " << message->radius <<  ", surface: " << message->surface << std::endl;
        std::cout << "\t Coefficients: [";
        for (int i = 0; i < 9; i++)
        {
          std::cout << message->coeffs[i];
          if (i < 8)
            std::cout << ", ";
        }
        std:: cout << "]" << std::endl;
        break;
      }
      case Message_Type::Surface:
      {
        std::cout << "Received Surface" << std::endl; /// TODO: Fill in surface data once it's being sent from LEPP
        break;
      }
      default:
      {
        std::cout << "UNKNOWN message type: " << header->type << "!!" << std::endl;
      }
    }

  }
  close(s_other);
  close(s);
}

int main(int argc, char* argv[])
{
  ParsedParams params;
  if (!parse_args(argc, argv, &params))
  {
    printHelp(argv[0]);
    return 0;
  }

  listen(params.port, params.verbose);

  return 0;
}

