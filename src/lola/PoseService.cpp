#include "lola/PoseService.h"
#include "lola/pose/PoseFileService.hpp"
#include "lola/pose/PoseUdpService.hpp"

std::shared_ptr<lepp::PoseService> PoseServiceFromUdp(std::string const& host, int port) {
  uint16_t p = static_cast<uint16_t>(port);
  assert(p == port);

  std::shared_ptr<lepp::PoseService> ps(new PoseUdpService(host, p));
  return ps;
}

std::shared_ptr<lepp::PoseService> PoseServiceFromFile(std::string const& filename) {
  std::shared_ptr<lepp::PoseService> ps(new PoseFileService(filename));
  return ps;
}
