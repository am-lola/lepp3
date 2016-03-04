/**
 * A sample program to demonstrate pointcloud recorder. The recorded data could
 * be used for offline development
 */
#include <iostream>
#include <sstream>
#include <vector>

#include <pcl/io/openni2_grabber.h>
#include <pcl/io/pcd_grabber.h>

#include "lepp3/BaseObstacleDetector.hpp"
#include "lepp3/GrabberVideoSource.hpp"
#include "lepp3/BaseVideoSource.hpp"
#include "lepp3/VideoObserver.hpp"
#include "lepp3/FilteredVideoSource.hpp"

#include "lepp3/filter/TruncateFilter.hpp"
#include "lepp3/filter/SensorCalibrationFilter.hpp"

#include "lepp3/util/VideoRecorder.hpp"

#include "lola/PoseService.h"

#include "lepp3/models/ObjectModel.h"
#include "deps/easylogging++.h"
_INITIALIZE_EASYLOGGINGPP

using namespace lepp;

namespace {
  typedef SimplePoint PointT;
}

/**
 * Simple struct to hold the recording options
 */
struct record_options {
  bool record_cloud;
  bool record_pose;
};
/**
  * convenience methods to parse the command line
  */
// defining enums (will be used as a workaround for the switch case for
// string comparison)
enum command_option {
  h,
  help,
  cloud,
  pose,
  t,
  f,
};
// hash function to convert string to enum
command_option hashOption(std::string const& s) {
  if(s == "-h") return h;
  if(s == "--help") return help;
  if(s == "--cloud") return cloud;
  if(s == "--pose") return pose;
  if(s == "true") return t;
  if(s == "false") return f;
}

/**
 * Prints out the expected CLI usage of the program.
 */
void PrintUsage(std::string name) {
  std::cout << "Usage: " << name << " <option(s)>\n";
  std::cout << "Options:\n"
            << "\t-h, --help\t\t\tShow this help message\n"
            << "\t--cloud [boolean]\t\tToggle saving pointcloud\n"
            << "\t--pose [boolean]\t\tToggle saving pose information"
            << std::endl;
}

/**
 * Parses the command line arguments and executes the actions accordingly
 */
bool parseCommand(int argc, char* argv[], record_options& op) {
  if(argc < 2) // program runs with default parameters
    return true;
  for(int i=1; i<argc; ++i) {
    std::string const arg = argv[i];
    switch (hashOption(arg)) {
      case h:
      case help:
        PrintUsage(argv[0]);
        return false;
      case cloud:
        if(i+1 < argc) {
          std::string const value = argv[++i];
          if(value == "false")
            op.record_cloud = false;
        }
        break;
      case pose:
        if(i+1 < argc) {
          std::string const value = argv[++i];
          if(value == "false") {
            op.record_pose = false;
          }
        }
        break;
      default:
        PrintUsage(argv[0]);
        return false;
    }
  }
  return true;
}

/**
 * Builds a `FilteredVideoSource` instance that wraps the given raw source.
 */
template<class PointT>
boost::shared_ptr<FilteredVideoSource<PointT> >
buildFilteredSource(
    boost::shared_ptr<VideoSource<PointT> > raw) {
  // Wrap the given raw source.
  boost::shared_ptr<FilteredVideoSource<PointT> > source(
      new SimpleFilteredVideoSource<PointT>(raw));
  // Now set the point filters that should be used.
  {
    double const a = 1.0117;
    double const b = -0.0100851;
    boost::shared_ptr<PointFilter<PointT> > filter(
        new SensorCalibrationFilter<PointT>(a, b));
    source->addFilter(filter);
  }
  {
    boost::shared_ptr<PointFilter<PointT> > filter(
        new TruncateFilter<PointT>(2));
    source->addFilter(filter);
  }
  // no need to add a RobotOdoTransformer instance. We are not trying to
  // process the pointcloud.

  return source;
}

int main(int argc, char* argv[]) {

  // by default both values are set to true, unless changed by the user
  record_options op;
  op.record_cloud = true;
  op.record_pose = true;

  bool resume = parseCommand(argc, argv, op);
  if(!resume) {
    cout << "terminating the program..." << std::endl;
    return 0;
  }
  // Obtain a raw live stream video source
  boost::shared_ptr<VideoSource<PointT> > raw_source(
      new LiveStreamSource<PointT>(true));
  // Wrap the raw source in a filter
  boost::shared_ptr<FilteredVideoSource<PointT> > source(
      buildFilteredSource(raw_source));
  // prepare the recorder
  boost::shared_ptr<VideoRecorder<PointT> > recorder(
      new VideoRecorder<PointT>());
  // Set the recording mode
  if(op.record_cloud & op.record_pose)
    recorder->setMode(lepp::MODE_CLOUD_IMAGE_POSE);
  else if(op.record_pose == false)
    recorder->setMode(lepp::MODE_CLOUD_IMAGE);

  // attach the recorder to the video source
  if(op.record_cloud) {
    source->attachObserver(recorder);
  }

  boost::shared_ptr<PoseService> pose_service(
    new PoseService("192.168.0.8", 53249));

  if(op.record_pose) {
    pose_service->attachObserver(recorder);
    pose_service->start();
  }
  // No need for a RobotAggregator, since we are not sending anything back
  // to Lola.


  // Starts capturing new frames and forwarding them to attached observers.
  source->open();

  std::cout << "Waiting forever..." << std::endl;
  std::cout << "(^C to exit)" << std::endl;
  while (true)
    boost::this_thread::sleep(boost::posix_time::milliseconds(8000));

  return 0;
}
