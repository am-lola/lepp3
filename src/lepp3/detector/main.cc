/**
 * A program performing detection of obstacles in the given file feed
 * and visualizing their approximations.
 */
#include <iostream>

#include <pcl/io/openni2_grabber.h>
#include <pcl/io/pcd_grabber.h>

#include "lepp3/Typedefs.hpp"
#include "lepp3/ObstacleDetector.hpp"
#include "lepp3/SurfaceDetector.hpp"
#include "config/HardcodedParser.hpp"
#include "config/FileConfigParser.hpp"
#include "lepp3/GrabberVideoSource.hpp"
#include "lepp3/VideoSource.hpp"
#include "lepp3/FilteredVideoSource.hpp"
#include "lepp3/SmoothObstacleAggregator.hpp"
#include "lepp3/ConvexHullDetector.hpp"
#include "lepp3/SurfaceTracker.hpp"
#include "lepp3/SplitApproximator.hpp"
#include "lepp3/MomentOfInertiaApproximator.hpp"

#include "lepp3/visualization/EchoObserver.hpp"
#include "lepp3/visualization/Visualizer.hpp"
#include "lepp3/visualization/ARVisualizer.hpp"

#include "lepp3/filter/TruncateFilter.hpp"
#include "lepp3/filter/SensorCalibrationFilter.hpp"

#include "lepp3/models/ObjectModel.h"

#include "lepp3/util/FileManager.hpp"

#include "deps/toml.h"
#include "deps/easylogging++.h"

 _INITIALIZE_EASYLOGGINGPP

using namespace lepp;

/**
 * Prints out the expected CLI usage of the program.
 */
void PrintUsage() {
  std::cout << "usage: lola --cfg <cfg-file> | ((--pcd <file> | --oni <file> | --stream) [--live]])"
      << std::endl;
  std::cout << "--cfg    : " << "configure the vision subsytem by reading the "
      << "given config file" << std::endl;
  std::cout << "--pcd    : " << "read the input from a .pcd file" << std::endl;
  std::cout << "--oni    : " << "read the input from an .oni file" << std::endl;
  std::cout << "--stream : " << "read the input from a live stream based on a"
      << " sensor attached to the computer" << std::endl;
  std::cout << "--live   : " << "whether kinematics data is obtained from the robot"
      << std::endl;
}

int main(int argc, char* argv[]) {
  _START_EASYLOGGINGPP(argc, argv);
  // Initialize the context container
  boost::shared_ptr<Parser<PointT> > parser;
  try {
    for (int i = 1; i < argc; ++i) {
      if (std::string(argv[i]) == "--cfg" && i != argc) {
        // Default to using the FileConfigParser if a `cfg` CLI parameter is
        // passed.
        parser.reset(new FileConfigParser<PointT>(argv[i + 1]));
      }
    }

    if (!parser) {
      // Fall back to trying to do a hardcoded parser if no config file given.
      parser.reset(new HardcodedParser<PointT>(argv, argc));
    }
  } catch (char const* exc) {
    std::cerr << exc << std::endl;
    PrintUsage();
    return 1;
  }

  // Get the video source and start it up
  parser->source()->open();

  while (true)
    boost::this_thread::sleep(boost::posix_time::milliseconds(8000));

  return 0;
}
