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
#include "lepp3/GrabberVideoSource.hpp"
#include "lepp3/VideoSource.hpp"
#include "lepp3/VideoObserver.hpp"
#include "lepp3/FilteredVideoSource.hpp"
#include "lepp3/SmoothObstacleAggregator.hpp"
#include "lepp3/ConvexHullDetector.hpp"
#include "lepp3/SurfaceTracker.hpp"

#include "lepp3/visualization/EchoObserver.hpp"
#include "lepp3/visualization/Visualizer.hpp"

#include "lepp3/filter/TruncateFilter.hpp"
#include "lepp3/filter/SensorCalibrationFilter.hpp"

#include "lepp3/models/ObjectModel.h"
#include "deps/easylogging++.h"

 _INITIALIZE_EASYLOGGINGPP

using namespace lepp;

/**
 * Prints out the expected CLI usage of the program.
 */
void PrintUsage() {
  std::cerr << "usage: detector [--pcd file | --oni file | --stream]"
      << std::endl;
  std::cerr << "--pcd    : " << "read the input from a .pcd file" << std::endl;
  std::cerr << "--oni    : " << "read the input from an .oni file" << std::endl;
  std::cerr << "--stream : " << "read the input from a live stream based on a"
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
        new TruncateFilter<PointT>(2)); // STEP SIZE --> 1 CM
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
        new LiveStreamSource<PointT>());
  } else if (option == "--pcd" && argc >= 3) {
    std::string const file_path = argv[2];
    boost::shared_ptr<pcl::Grabber> interface(new pcl::PCDGrabber<PointT>(
      file_path,
      20.,
      true));
    return boost::shared_ptr<VideoSource<PointT> >(
        new GeneralGrabberVideoSource<PointT>(interface));
  } else if (option == "--oni" && argc >= 3) {
    std::string const file_path = argv[2];
    boost::shared_ptr<pcl::Grabber> interface(new pcl::io::OpenNI2Grabber(
      file_path,
      pcl::io::OpenNI2Grabber::OpenNI_Default_Mode,
      pcl::io::OpenNI2Grabber::OpenNI_Default_Mode));
    return boost::shared_ptr<VideoSource<PointT> >(
        new GeneralGrabberVideoSource<PointT>(interface));
  }

  // Unknown option: return a "null" pointer.
  return boost::shared_ptr<VideoSource<PointT> >();
}




/*
int main()
{
  ConvexHullDetector *chd = new ConvexHullDetector();

  PointCloudPtr hull = boost::shared_ptr<PointCloudT>(new PointCloudT());

  hull->push_back(*(new PointT(0,0,0)));
  hull->push_back(*(new PointT(0,1,0)));
  hull->push_back(*(new PointT(1,2,0)));
  hull->push_back(*(new PointT(3,2,0)));
  hull->push_back(*(new PointT(3.5,1.5,0)));
  hull->push_back(*(new PointT(3.5,1,0)));
  hull->push_back(*(new PointT(2,-1.5,0)));

  chd->reduceConvHullPoints(hull, 3);
}
*/




int main(int argc, char* argv[]) {
  // Obtain a video source based on the command line arguments received
  boost::shared_ptr<VideoSource<PointT> > raw_source(GetVideoSource(argc, argv));
  if (!raw_source) {
    PrintUsage();
    return 1;
  }

  //~~~~~~~~~~~~~~~~~~~~~~~Surfaces~~~~~~~~~~~~~~~~~~~~~~~~
  // Wrap the raw source in a filter
  boost::shared_ptr<FilteredVideoSource<PointT> > source(
      buildFilteredSource(raw_source));

  boost::shared_ptr<SurfaceDetector<PointT> > surfaceDetector(
      new SurfaceDetector<PointT>());
  source->attachObserver(surfaceDetector);

  // Surface Post Processing
  boost::shared_ptr<SurfaceTracker<PointT> > post_surface_processor(
          new SurfaceTracker<PointT>);

  surfaceDetector->attachObserver(post_surface_processor);


    // add the surface convex hull detector
  boost::shared_ptr<ConvexHullDetector> convHullDetector(
    new ConvexHullDetector());
  post_surface_processor->attachObserver(convHullDetector);

//~~~~~~~~~~~~~~~~~~~~~~~Obstacles~~~~~~~~~~~~~~~~~~~~~~~~
// Prepare the approximator that the detector is to use.
  // First, the simple approximator...
  boost::shared_ptr<ObjectApproximator<PointT> > obstacle_simple_approx(
      boost::shared_ptr<ObjectApproximator<PointT> >(
        new MomentOfInertiaObjectApproximator<PointT>));
  // ...then the split strategy
  boost::shared_ptr<CompositeSplitStrategy<PointT> > obstacleSplitter(
      new CompositeSplitStrategy<PointT>);
  obstacleSplitter->addSplitCondition(boost::shared_ptr<SplitCondition<PointT> >(
      new DepthLimitSplitCondition<PointT>(1)));
  // ...finally, wrap those into a `SplitObjectApproximator` that is given
  // to the detector.
  boost::shared_ptr<ObjectApproximator<PointT> > obstacleApprox(
      new SplitObjectApproximator<PointT>(obstacle_simple_approx, obstacleSplitter));
  // Prepare the detector
  boost::shared_ptr<ObstacleDetector<PointT> > obstacleDetector(
      new ObstacleDetector<PointT>(obstacleApprox));
  // Attaching the detector to the source: process the point clouds obtained
  // by the source.
  surfaceDetector->attachObserver(obstacleDetector);

  // The visualizer is additionally decorated by the "smoothener" to smooth out
  // the output...
  boost::shared_ptr<SmoothObstacleAggregator> smooth_decorator(
      new SmoothObstacleAggregator);
  obstacleDetector->attachObstacleAggregator(smooth_decorator);


//~~~~~~~~~~~~~~~~~~~~~~~Visualizer~~~~~~~~~~~~~~~~~~~~~~~~
  // Prepare the result visualizer...
  boost::shared_ptr<Visualizer<PointT> > pclVisualizer(
    new Visualizer<PointT>());

  // Attaching the visualizer to the source: allow it to display the original point cloud.
  source->attachObserver(pclVisualizer);
  post_surface_processor->attachObserver(pclVisualizer);
  convHullDetector->attachObserver(pclVisualizer);
  smooth_decorator->attachObstacleAggregator(pclVisualizer);
  
  // Starts capturing new frames and forwarding them to attached observers.
  source->open();

  while (true)
    boost::this_thread::sleep(boost::posix_time::milliseconds(8000));

  return 0;
}
