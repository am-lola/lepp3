/**
 * A program performing detection of obstacles in the given file feed
 * and visualizing their approximations.
 */
#include <iostream>

#include <pcl/io/openni2_grabber.h>
#include <pcl/io/pcd_grabber.h>

#include "lepp3/util/FileManager.hpp"

#include "lepp3/BaseObstacleDetector.hpp"
#include "lepp3/GrabberVideoSource.hpp"
#include "lepp3/BaseVideoSource.hpp"
#include "lepp3/VideoObserver.hpp"
#include "lepp3/FilteredVideoSource.hpp"
#include "lepp3/SmoothObstacleAggregator.hpp"

#include "lepp3/visualization/EchoObserver.hpp"
#include "lepp3/visualization/ObstacleVisualizer.hpp"
#include "lepp3/visualization/StairVisualizer.hpp"

#include "lepp3/filter/TruncateFilter.hpp"
#include "lepp3/filter/SensorCalibrationFilter.hpp"

#include "lepp3/models/ObjectModel.h"
#include "deps/easylogging++.h"
_INITIALIZE_EASYLOGGINGPP


using namespace lepp;

namespace {
  typedef SimplePoint PointT;
}

/**
 * Prints out the expected CLI usage of the program.
 */
void PrintUsage() {
  std::cout << "usage: detector [--pcd file | --oni file | --stream]"
      << std::endl;
  std::cout << "--pcd    : " << "read the input from a .pcd file" << std::endl;
  std::cout << "--oni    : " << "read the input from an .oni file" << std::endl;
  std::cout << "--stream : " << "read the input from a live stream based on a"
      << " sensor attached to the computer" << std::endl;
}

/**
 * Builds a `FilteredVideoSource` instance that wraps the given raw source.
 */
template<class PointT>
boost::shared_ptr<FilteredVideoSource<PointT> >
buildFilteredSource(boost::shared_ptr<VideoSource<PointT> > raw) {
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

  return source;
}

/**
 * Parses the command line arguments received by the program and chooses
 * the appropriate video source based on those.
 */
boost::shared_ptr<VideoSource<PointT> > GetVideoSource(int argc, char* argv[]) {
  if (argc < 2) {
    return boost::shared_ptr<VideoSource<PointT> >();
  }

  std::string const option = argv[1];
  if (option == "--stream") {
    return boost::shared_ptr<VideoSource<PointT> >(
        new LiveStreamSource<PointT>(true));
  } else if (option == "--pcd" && argc >= 3) {
    std::string const directory_path = argv[2];
    FileManager fm(directory_path);
    const std::vector<std::string> file_names = fm.getFileNames(".pcd");
    boost::shared_ptr<pcl::Grabber> interface(new pcl::PCDGrabber<PointT>(
      file_names,
      30.,
      true));
    return boost::shared_ptr<VideoSource<PointT> >(
        new GeneralGrabberVideoSource<PointT>(interface, true));
  } else if (option == "--oni" && argc >= 3) {
    std::string const file_path = argv[2];
    boost::shared_ptr<pcl::Grabber> interface(new pcl::io::OpenNI2Grabber(
      file_path,
      pcl::io::OpenNI2Grabber::OpenNI_Default_Mode,
      pcl::io::OpenNI2Grabber::OpenNI_Default_Mode));
    return boost::shared_ptr<VideoSource<PointT> >(
        new GeneralGrabberVideoSource<PointT>(interface, true));
  }

  // Unknown option: return a "null" pointer.
  return boost::shared_ptr<VideoSource<PointT> >();
}

int main(int argc, char* argv[]) {
  // Obtain a video source based on the command line arguments received
  boost::shared_ptr<VideoSource<PointT> > raw_source(GetVideoSource(argc, argv));
  if (!raw_source) {
    PrintUsage();
    return 1;
  }
  // Wrap the raw source in a filter
  boost::shared_ptr<FilteredVideoSource<PointT> > source(
      buildFilteredSource(raw_source));

  // Prepare the approximator that the detector is to use.
  // First, the simple approximator...
  boost::shared_ptr<ObjectApproximator<PointT> > simple_approx(
      boost::shared_ptr<ObjectApproximator<PointT> >(
        new MomentOfInertiaObjectApproximator<PointT>));
  // ...then the split strategy
  boost::shared_ptr<CompositeSplitStrategy<PointT> > splitter(
      new CompositeSplitStrategy<PointT>);
  splitter->addSplitCondition(boost::shared_ptr<SplitCondition<PointT> >(
      new DepthLimitSplitCondition<PointT>(1)));
  // ...finally, wrap those into a `SplitObjectApproximator` that is given
  // to the detector.
  boost::shared_ptr<ObjectApproximator<PointT> > approx(
      new SplitObjectApproximator<PointT>(simple_approx, splitter));
  // Prepare the detector
  boost::shared_ptr<BaseObstacleDetector<PointT> > detector(
      new BaseObstacleDetector<PointT>(approx));
  // Attaching the detector to the source: process the point clouds obtained
  // by the source.
  //source->attachObserver(detector);

  // Prepare the result visualizer...
  boost::shared_ptr<StairVisualizer<PointT> > visualizer(
      new StairVisualizer<PointT>());
  // Attaching the visualizer to the source: allow it to display the original
  // point cloud.
  source->attachObserver(visualizer);
  // The visualizer is additionally decorated by the "smoothener" to smooth out
  // the output...
  // boost::shared_ptr<SmoothObstacleAggregator> smooth_decorator(
  //     new SmoothObstacleAggregator);
  // detector->attachObstacleAggregator(smooth_decorator);
  // smooth_decorator->attachObstacleAggregator(visualizer);

  // Starts capturing new frames and forwarding them to attached observers.
  source->open();

  std::cout << "Waiting forever..." << std::endl;
  std::cout << "(^C to exit)" << std::endl;
  while (true)
    boost::this_thread::sleep(boost::posix_time::milliseconds(8000));

  return 0;
}
