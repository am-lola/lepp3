#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <signal.h>

#include <getopt.h>
#include <algorithm>
#include <string>
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>

#include <iface_vision_msg.hpp>

using am2b::iface::VisionMessageHeader;
using am2b::iface::Message_Type;
using am2b::iface::VisionMessage;
using am2b::iface::ObstacleMessage;
using am2b::iface::ObstacleType;
using am2b::iface::SurfaceMessage;



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
  int c;

  if (argc < 3)
  {
    std::cout << "ERROR: You must provide at least a port and an IP address!" << std::endl;
    return false;
  }

  while (1)
  {
    static struct option long_options[] =
    {
      {"port",    required_argument, 0, 'p'},
      {"host",    required_argument, 0, 'i'},
      {"verbose", no_argument,       0, 'v'},
      {"help",    no_argument,       0, '?'},
      {0}
    };

    int option_index = 0;
    c = getopt_long(argc, argv, "p:h:v?",
                    long_options, &option_index);

    /* detect end of options */
    if (c == -1)
      break;

    switch (c)
    {
      case 'p':
          params->port = std::stoi(optarg);
          break;

      case 'h':
          params->host_ip = std::string(optarg);
          break;

      case 'v':
          params->verbose = true;
          break;

      case '?':
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
  std::cout << "\t" << name << " --port Port-Number --host hostname [--verbose] [--help]" << std::endl;
  std::cout << "\t" << name << " -p Port-Number -h hostname [-v] [-?]" << std::endl;
  std::cout << "\t" << std::endl;
  std::cout << "\t" << "    port: TCP Port to use" << std::endl;
  std::cout << "\t" << "    host: Remote host (name or address) to connect to" << std::endl;
  std::cout << "\t" << "    help: Display this message" << std::endl;
  std::cout << std::endl;
}

void failWithError(std::string s)
{
  perror(s.c_str());
  exit(1);
}


// connects to hostname:port, and returns an open socket on success
int connect(unsigned int port, std::string hostname, bool verbose)
{
  char port_str[16];
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

    if (s < 0)
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
      perror("Could not connect");
    }

    close(s); // if we failed, close connection and try the next one
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
  // ignore SIGPIPE so we can print errors on write fails
  signal(SIGPIPE, SIG_IGN);

  // obstacles to send
  std::vector<double> sphere_coeffs  = {2.0, 2.0, 0.0,  0.0,  0.0,  0.0, 0.0, 0.0, 0.0};
  std::vector<double> capsule_coeffs = {1.0, 1.0, 1.0, -1.0, -1.0, -1.0, 0.0, 0.0, 0.0};
  std::vector<VisionMessage> messages = {
    VisionMessage(ObstacleMessage::SetMessage(ObstacleType::Sphere, 0, 1, 0.1, sphere_coeffs)),
    VisionMessage(ObstacleMessage::SetMessage(ObstacleType::Capsule, 1, 1, 0.15, capsule_coeffs)),
    VisionMessage(ObstacleMessage::ModifyMessage(ObstacleType::Sphere, 0, 1, 0.2, sphere_coeffs)),
    VisionMessage(ObstacleMessage::ModifyMessage(ObstacleType::Capsule, 1, 1, 0.05, capsule_coeffs)),
    VisionMessage(ObstacleMessage::DeleteMessage(0)),
    VisionMessage(ObstacleMessage::DeleteMessage(1))
  };

  // loop over messages, sending one each iteration, and repeating once we reach the end of the list
  for (int i = 0; /* loop forever */; i = (i+1)%messages.size())
  {
    int sent = 0;
    VisionMessage msg = messages[i];
    unsigned char* buf = (unsigned char*)(&msg);

    std::cout << std::endl << "Sending message: " << msg << std::endl;

     // send the header
    sent = write(s, buf, sizeof(VisionMessageHeader));
    if (sent <= 0)
      failWithError("Failed to send data!");

    if (verbose)
    {
      std::cout << "Sent " << sent << " bytes (message header)" << std::endl;
    }

     // send the message content
    sent = write(s, msg.content, msg.header.len);
    if (sent <= 0)
      failWithError("Failed to send data!");

    if (verbose)
    {
      std::cout << "Sent " << sent << " bytes (message content)" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::seconds(2)); /// TODO: This should be configurable from cmd line
  }
}

int main(int argc, char* argv[])
{
  ParsedParams params;
  if (!parse_args(argc, argv, &params))
  {
    printHelp(argv[0]);
    return 0;
  }

  int socket = connect(params.port, params.host_ip, params.verbose);
  transmit(socket, params.verbose);
  close(socket);
  return 0;
}
