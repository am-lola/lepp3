#ifndef LOLA_POSE_SERVICE_H__
#define LOLA_POSE_SERVICE_H__

#include <memory>
#include "lepp3/pose/PoseService.hpp"

/**
 * Creates a Pose service which listens on a UDP port
 */
std::shared_ptr<lepp::PoseService> PoseServiceFromUdp(std::string const& host, int port);
/**
 * Creates a Pose service from a saved file
 */
std::shared_ptr<lepp::PoseService> PoseServiceFromFile(std::string const& filename);

#endif
